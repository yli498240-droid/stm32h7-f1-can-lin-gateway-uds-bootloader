/* F103C8 USART3 LIN Slave over the external TJA1021 physical layer. */
#include "lin_slave.h"
#include "button.h"
#include "lin_nm_slave.h"
#include "lin_phy.h"
#include "lin_diag_slave.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

volatile LinSlave_t g_lin = {0};

#define LIN_FRAME_TIMEOUT_MS       10U
#define LIN_DIAG_LOG_PERIOD_MS   1000U
#define LIN_BUTTON_QUEUE_CAPACITY   8U

typedef struct {
    uint8_t event;
    uint8_t sequence;
} LinButtonEventEntry_t;

static uint32_t s_diag_log_tick = 0U;
static volatile LinButtonEventEntry_t s_button_queue[LIN_BUTTON_QUEUE_CAPACITY];
static volatile uint8_t s_button_read_index = 0U;
static volatile uint8_t s_button_write_index = 0U;
static volatile uint8_t s_button_count = 0U;
static volatile uint8_t s_next_button_sequence = 1U;
static volatile uint8_t s_led_activity = 0U;
static volatile uint8_t s_led_last_tx_sequence = 0U;
static volatile uint32_t s_button_drop_count = 0U;
static volatile uint32_t s_button_ack_count = 0U;
static volatile uint32_t s_button_accept_count = 0U;
static volatile uint32_t s_lock_reject_count = 0U;
static volatile uint8_t s_window_switch_enabled = 1U;
static volatile uint8_t s_antipinch_warning = 0U;
static volatile uint8_t s_status_alive_counter = 0U;
static volatile uint32_t s_transport_arm_count = 0U;

uint8_t LIN_Slave_IsUartReady(void)
{
    const uint32_t required_cr1 = USART_CR1_UE | USART_CR1_TE |
                                  USART_CR1_RE | USART_CR1_RXNEIE;
    const uint32_t required_cr2 = USART_CR2_LINEN | USART_CR2_LBDL |
                                  USART_CR2_LBDIE;

    return (((huart3.Instance->CR1 & required_cr1) == required_cr1) &&
            ((huart3.Instance->CR2 & required_cr2) == required_cr2)) ? 1U : 0U;
}

void LIN_Slave_ArmAwakeTransport(void)
{
    uint32_t primask = __get_PRIMASK();
    volatile uint32_t discard;

    __disable_irq();

    /* 复用 M12.6 直接 IRQ parser，只在 Cold Boot/Wake 边界做确定性
     * arm：先 mask，再清理旧 SR/DR/LBD/NVIC，最后开 RXNE/LBD。 */
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
    CLEAR_BIT(huart3.Instance->CR2, USART_CR2_LBDIE);
    SET_BIT(huart3.Instance->CR1, USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
    SET_BIT(huart3.Instance->CR2, USART_CR2_LINEN | USART_CR2_LBDL);

    discard = huart3.Instance->SR;
    discard = huart3.Instance->DR;
    (void)discard;
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_LBD);
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);

    g_lin.state = LIN_STATE_IDLE;
    g_lin.data_idx = 0U;
    g_lin.data_len = 0U;

    SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
    SET_BIT(huart3.Instance->CR2, USART_CR2_LBDIE);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    s_transport_arm_count++;

    if (primask == 0U) __enable_irq();
}

void LIN_Slave_SuspendTransport(void)
{
    uint32_t primask = __get_PRIMASK();
    volatile uint32_t discard;

    __disable_irq();
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE | USART_CR1_RE);
    CLEAR_BIT(huart3.Instance->CR2, USART_CR2_LBDIE);
    discard = huart3.Instance->SR;
    discard = huart3.Instance->DR;
    (void)discard;
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_LBD);
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
    g_lin.state = LIN_STATE_IDLE;
    g_lin.data_idx = 0U;
    g_lin.data_len = 0U;
    if (primask == 0U) __enable_irq();
}

static void lin_slave_recover(void)
{
    g_lin.state = LIN_STATE_IDLE;
    g_lin.data_idx = 0U;
    g_lin.data_len = 0U;
    g_lin.recovery_cnt++;

    /* 每条异常路径都回到同一个 WAIT_BREAK 硬件入口。 */
    huart3.Instance->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
    huart3.Instance->CR1 |= USART_CR1_UE | USART_CR1_TE;
    huart3.Instance->CR2 |= USART_CR2_LINEN | USART_CR2_LBDL |
                            USART_CR2_LBDIE;
}

uint8_t LIN_Slave_QueueButtonEvent(uint8_t event)
{
    uint32_t primask;

    if ((event < 0x01U) || (event > 0x05U) ||
        !LinNmSlave_IsNetworkAwake()) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_window_switch_enabled == 0U) {
        s_lock_reject_count++;
        if (primask == 0U) __enable_irq();
        return 0U;
    }
    if (s_button_count >= LIN_BUTTON_QUEUE_CAPACITY) {
        s_button_drop_count++;
        if (primask == 0U) __enable_irq();
        return 0U;
    }

    s_button_queue[s_button_write_index].event = event;
    s_button_queue[s_button_write_index].sequence = s_next_button_sequence;
    s_button_write_index =
        (uint8_t)((s_button_write_index + 1U) % LIN_BUTTON_QUEUE_CAPACITY);
    s_button_count++;
    s_button_accept_count++;
    s_next_button_sequence++;
    if (s_next_button_sequence == 0U) s_next_button_sequence = 1U;
    if (primask == 0U) __enable_irq();
    return 1U;
}

void LIN_Slave_ClearButtonEvents(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_button_read_index = 0U;
    s_button_write_index = 0U;
    s_button_count = 0U;
    s_led_activity = 0U;
    if (primask == 0U) __enable_irq();
}

uint8_t LIN_Slave_GetPendingButtonCount(void)
{
    return s_button_count;
}

uint8_t LIN_Slave_TakeLedActivity(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t activity;

    __disable_irq();
    activity = s_led_activity;
    s_led_activity = 0U;
    if (primask == 0U) __enable_irq();
    return activity;
}

uint32_t LIN_Slave_GetButtonDropCount(void)
{
    return s_button_drop_count;
}

void LIN_Slave_GetDiagSnapshot(LinSlave_DiagSnapshotType *snapshot)
{
    if (snapshot == NULL) return;
    snapshot->err_sync = g_lin.err_sync;
    snapshot->err_pid = g_lin.err_pid;
    snapshot->err_checksum = g_lin.err_chk;
    snapshot->err_timeout = g_lin.err_timeout;
    snapshot->err_uart = g_lin.err_uart;
    snapshot->recovery_count = g_lin.recovery_cnt;
    snapshot->button_accepted = s_button_accept_count;
    snapshot->button_rejected = s_lock_reject_count;
    snapshot->button_dropped = s_button_drop_count;
    snapshot->pending_count = s_button_count;
    snapshot->window_switch_enabled = s_window_switch_enabled;
    snapshot->antipinch_warning = s_antipinch_warning;
    snapshot->alive_counter = s_status_alive_counter;
}

void LIN_Slave_ClearDiagnosticStatistics(void)
{
    /* Never touch FIFO indices/count, sequence/ACK, alive or current states. */
    g_lin.rx_frame_cnt = 0U;
    g_lin.rx_21_cnt = 0U;
    g_lin.tx_22_cnt = 0U;
    g_lin.tx_23_cnt = 0U;
    g_lin.err_cnt = 0U;
    g_lin.err_sync = 0U;
    g_lin.err_pid = 0U;
    g_lin.err_chk = 0U;
    g_lin.err_timeout = 0U;
    g_lin.err_uart = 0U;
    g_lin.err_ore = 0U;
    g_lin.err_fe = 0U;
    g_lin.err_ne = 0U;
    g_lin.err_pe = 0U;
    g_lin.break_cnt = 0U;
    g_lin.recovery_cnt = 0U;
    s_button_accept_count = 0U;
    s_button_drop_count = 0U;
    s_button_ack_count = 0U;
    s_lock_reject_count = 0U;
}

static void lin_button_acknowledge(uint8_t acknowledged_sequence)
{
    if ((s_button_count != 0U) &&
        (s_button_queue[s_button_read_index].sequence ==
         acknowledged_sequence)) {
        s_button_read_index =
            (uint8_t)((s_button_read_index + 1U) % LIN_BUTTON_QUEUE_CAPACITY);
        s_button_count--;
        s_button_ack_count++;
    }
}

uint8_t LIN_CalcPID(uint8_t id)
{
    uint8_t b0 = (id >> 0) & 1;
    uint8_t b1 = (id >> 1) & 1;
    uint8_t b2 = (id >> 2) & 1;
    uint8_t b3 = (id >> 3) & 1;
    uint8_t b4 = (id >> 4) & 1;
    uint8_t b5 = (id >> 5) & 1;
    uint8_t p0 = b0 ^ b1 ^ b2 ^ b4;
    uint8_t p1 = !(b1 ^ b3 ^ b4 ^ b5);
    return (id & 0x3F) | (p0 << 6) | (p1 << 7);
}

uint8_t LIN_CalcEnhancedChecksum(uint8_t pid, const uint8_t *data, uint8_t len)
{
    uint16_t sum = pid;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
        if (sum >= 0x100) sum -= 0xFF;
    }
    return (uint8_t)(~sum);
}

void LIN_Slave_Init(void)
{
    g_lin.state = LIN_STATE_IDLE;
    g_lin.rx_frame_cnt = 0;
    g_lin.rx_21_cnt = 0;
    g_lin.tx_23_cnt = 0;
    g_lin.tx_22_cnt = 0;
    g_lin.err_cnt = 0;
    g_lin.err_sync = 0;
    g_lin.err_pid = 0;
    g_lin.err_chk = 0;
    g_lin.err_timeout = 0;
    g_lin.err_uart = 0U;
    g_lin.err_ore = 0U;
    g_lin.err_fe = 0U;
    g_lin.err_ne = 0U;
    g_lin.err_pe = 0U;
    g_lin.break_cnt = 0U;
    g_lin.recovery_cnt = 0U;
    g_lin.last_activity_tick = HAL_GetTick();
    g_lin.last_21[0] = 0U;
    g_lin.last_21[1] = 0U;
    g_lin.last_21[2] = 0U;
    g_lin.last_23[0] = 0U;
    g_lin.last_23[1] = 0U;
    g_lin.last_23[2] = 0U;
    g_lin.last_23[3] = 0U;
    s_button_read_index = 0U;
    s_button_write_index = 0U;
    s_button_count = 0U;
    s_next_button_sequence = 1U;
    s_button_drop_count = 0U;
    s_button_ack_count = 0U;
    s_button_accept_count = 0U;
    s_lock_reject_count = 0U;
    s_window_switch_enabled = 1U;
    s_antipinch_warning = 0U;
    s_status_alive_counter = 0U;
    s_transport_arm_count = 0U;
    s_led_activity = 0U;
    s_diag_log_tick = HAL_GetTick();
    printf("[LIN] Slave Init OK (USART3 19200bps, TJA1021)\r\n");
}

static uint8_t lin_calc_classic_checksum(const uint8_t *data, uint8_t len)
{
    uint16_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < len; i++) {
        sum += data[i];
        if (sum >= 0x100U) sum -= 0xFFU;
    }
    return (uint8_t)(~sum);
}

void LIN_Slave_OnBreak(void)
{
    if (!LinNmSlave_IsProtocolEnabled()) {
        g_lin.state = LIN_STATE_IDLE;
        return;
    }
    g_lin.break_cnt++;
    if (g_lin.state != LIN_STATE_IDLE) {
        g_lin.recovery_cnt++;
    }
    g_lin.state    = LIN_STATE_SYNC;
    g_lin.data_idx = 0;
    g_lin.data_len = 0;
    g_lin.last_activity_tick = HAL_GetTick();
}

void LIN_Slave_OnUartError(uint32_t usart_sr, uint8_t bad_byte)
{
    (void)bad_byte;
    if (!LinNmSlave_IsProtocolEnabled()) {
        g_lin.state = LIN_STATE_IDLE;
        return;
    }
    g_lin.err_cnt++;
    g_lin.err_uart++;
    if ((usart_sr & USART_SR_ORE) != 0U) g_lin.err_ore++;
    if ((usart_sr & USART_SR_FE)  != 0U) g_lin.err_fe++;
    if ((usart_sr & USART_SR_NE)  != 0U) g_lin.err_ne++;
    if ((usart_sr & USART_SR_PE)  != 0U) g_lin.err_pe++;
    lin_slave_recover();
}

static uint8_t pid_to_data_len(uint8_t pid)
{
    switch (pid) {
        case LIN_PID_WINDOW_CMD: return 3;
        case LIN_PID_BTN_REPORT: return 2;
        case LIN_PID_STATUS:     return 4;
        case LIN_PID_MASTER_REQUEST: return 8;
        default:                 return 0;
    }
}

static void lin_slave_send_btn_response(void)
{
    uint8_t pid = LIN_PID_BTN_REPORT;
    uint8_t data[2] = {0U, 0U};
    uint8_t cs;

    if (s_button_count != 0U) {
        data[0] = s_button_queue[s_button_read_index].event;
        data[1] = s_button_queue[s_button_read_index].sequence;
    }
    cs = LIN_CalcEnhancedChecksum(pid, data, 2U);

    /* === 关键:发送期间临时关 RE,避免自己听到自己 === */
    huart3.Instance->CR1 &= ~USART_CR1_RE;

    while (!(huart3.Instance->SR & USART_SR_TXE));
    huart3.Instance->DR = data[0];

    while (!(huart3.Instance->SR & USART_SR_TXE));
    huart3.Instance->DR = data[1];
    
    while (!(huart3.Instance->SR & USART_SR_TXE));
    huart3.Instance->DR = cs;
    
    while (!(huart3.Instance->SR & USART_SR_TC));

    /* 清掉发完瞬间可能进 RX 寄存器的垃圾 */
    volatile uint32_t tmp = huart3.Instance->SR;
    tmp = huart3.Instance->DR;
    (void)tmp;

    /* 重新打开 RE */
    huart3.Instance->CR1 |= USART_CR1_RE;

    g_lin.tx_22_cnt++;
    /* 同一 FIFO 头在 ACK 前可能被 20 ms polling 重传；LED 只为一个新的
     * Event Sequence 触发一次 TX 动画，Idle 和协议重传都不重复触发。 */
    if ((data[0] != 0U) && (data[1] != s_led_last_tx_sequence)) {
        s_led_last_tx_sequence = data[1];
        s_led_activity |= LIN_LED_ACTIVITY_TX;
    }
}

static void lin_slave_send_status_response(void)
{
    uint8_t data[4] = {0U, 0U, 0U, 0U};
    uint8_t cs;
    volatile uint32_t tmp;

    data[0] = Button_GetPhysicalState();
    if (s_window_switch_enabled == 0U) data[0] |= (1U << 3);
    data[1] = (uint8_t)(s_button_count & 0x0FU);
    if (s_button_drop_count != 0U) data[1] |= (1U << 7);
    if (g_lin.recovery_cnt != 0U) data[2] |= (1U << 0);
    if (g_lin.err_uart != 0U) data[2] |= (1U << 1);
    if (g_lin.err_chk != 0U) data[2] |= (1U << 2);
    if (g_lin.err_timeout != 0U) data[2] |= (1U << 3);
    data[3] = s_status_alive_counter++;
    cs = LIN_CalcEnhancedChecksum(LIN_PID_STATUS, data, 4U);

    huart3.Instance->CR1 &= ~USART_CR1_RE;
    while ((huart3.Instance->SR & USART_SR_TXE) == 0U) { }
    huart3.Instance->DR = data[0];
    while ((huart3.Instance->SR & USART_SR_TXE) == 0U) { }
    huart3.Instance->DR = data[1];
    while ((huart3.Instance->SR & USART_SR_TXE) == 0U) { }
    huart3.Instance->DR = data[2];
    while ((huart3.Instance->SR & USART_SR_TXE) == 0U) { }
    huart3.Instance->DR = data[3];
    while ((huart3.Instance->SR & USART_SR_TXE) == 0U) { }
    huart3.Instance->DR = cs;
    while ((huart3.Instance->SR & USART_SR_TC) == 0U) { }

    tmp = huart3.Instance->SR;
    tmp = huart3.Instance->DR;
    (void)tmp;
    huart3.Instance->CR1 |= USART_CR1_RE;

    g_lin.last_23[0] = data[0];
    g_lin.last_23[1] = data[1];
    g_lin.last_23[2] = data[2];
    g_lin.last_23[3] = data[3];
    g_lin.tx_23_cnt++;
}

static void lin_slave_send_diag_response(void)
{
    uint8_t data[8];
    uint8_t cs;
    uint8_t i;
    volatile uint32_t tmp;

    if (LinDiagSlave_TakeResponse(data) == 0U) return;
    cs = lin_calc_classic_checksum(data, 8U);
    huart3.Instance->CR1 &= ~USART_CR1_RE;
    for (i = 0U; i < 8U; i++) {
        while ((huart3.Instance->SR & USART_SR_TXE) == 0U) { }
        huart3.Instance->DR = data[i];
    }
    while ((huart3.Instance->SR & USART_SR_TXE) == 0U) { }
    huart3.Instance->DR = cs;
    while ((huart3.Instance->SR & USART_SR_TC) == 0U) { }
    tmp = huart3.Instance->SR;
    tmp = huart3.Instance->DR;
    (void)tmp;
    huart3.Instance->CR1 |= USART_CR1_RE;
}

static void lin_handle_frame(void)
{
    uint8_t expected = (g_lin.pid == LIN_PID_MASTER_REQUEST) ?
        lin_calc_classic_checksum((uint8_t*)g_lin.data, g_lin.data_len) :
        LIN_CalcEnhancedChecksum(
            g_lin.pid, (uint8_t*)g_lin.data, g_lin.data_len);

    if (expected != g_lin.checksum) {
        g_lin.err_cnt++;
        g_lin.err_chk++;
        lin_slave_recover();
        return;
    }

    g_lin.rx_frame_cnt++;

    if (g_lin.pid == LIN_PID_MASTER_REQUEST) {
        uint8_t i;
        uint8_t valid_gts = (g_lin.data[0] == 0x00U) ? 1U : 0U;

        for (i = 1U; i < 8U; i++) {
            if (g_lin.data[i] != 0xFFU) valid_gts = 0U;
        }
        if (valid_gts != 0U) {
            LinDiagSlave_Abort();
            LinNmSlave_NotifyGoToSleepFromIsr();
        } else {
            LinDiagSlave_OnMasterRequest((const uint8_t *)g_lin.data);
        }
    } else if (g_lin.pid == LIN_PID_WINDOW_CMD) {
        uint8_t previous_switch_enabled = s_window_switch_enabled;
        uint8_t previous_antipinch = s_antipinch_warning;

        g_lin.last_21[0] = g_lin.data[0];
        g_lin.last_21[1] = g_lin.data[1];
        g_lin.last_21[2] = g_lin.data[2];
        g_lin.rx_21_cnt++;
        s_window_switch_enabled =
            ((g_lin.data[0] & (1U << 0)) != 0U) ? 1U : 0U;
        s_antipinch_warning =
            ((g_lin.data[0] & (1U << 1)) != 0U) ? 1U : 0U;
        lin_button_acknowledge(g_lin.data[2]);
        LinNmSlave_NotifyValidControlFromIsr();

        if (s_window_switch_enabled != previous_switch_enabled) {
            if (s_window_switch_enabled == 0U) {
                /* LOCK 切入时丢弃所有未 ACK 事件，禁止解锁后补发旧动作。 */
                s_button_read_index = 0U;
                s_button_write_index = 0U;
                s_button_count = 0U;
            }
            s_led_activity |= LIN_LED_ACTIVITY_LOCK_CHANGE;
        }
        if ((s_antipinch_warning != 0U) &&
            (previous_antipinch == 0U)) {
            s_led_activity |= LIN_LED_ACTIVITY_ANTIPINCH;
        }
        s_led_activity |= LIN_LED_ACTIVITY_RX;
    }
}

void LIN_Slave_OnByte(uint8_t b)
{
    if (!LinNmSlave_IsProtocolEnabled()) {
        return;
    }
    g_lin.last_activity_tick = HAL_GetTick();

    /* === 关键:任何状态下收到 0x55,都尝试当 Sync 重新对齐 === */
    if (b == 0x55 && (g_lin.state == LIN_STATE_IDLE || g_lin.state == LIN_STATE_SYNC)) {
        g_lin.state = LIN_STATE_PID;
        return;
    }
    
    switch (g_lin.state) {

    case LIN_STATE_IDLE:
        break;

    case LIN_STATE_SYNC:
        /* 走到这只可能 b != 0x55,直接回 IDLE 等下一个 break */
        g_lin.err_cnt++;
        g_lin.err_sync++;
        lin_slave_recover();
        break;

    case LIN_STATE_PID:
        g_lin.pid = b;
        g_lin.id  = b & 0x3F;

        if (LIN_CalcPID(g_lin.id) != b) {
            g_lin.err_cnt++;
            g_lin.err_pid++;
            lin_slave_recover();
            break;
        }

        if (b == LIN_PID_SLAVE_RESPONSE) {
            lin_slave_send_diag_response();
            g_lin.state = LIN_STATE_IDLE;
            break;
        }

        g_lin.data_len = pid_to_data_len(b);
        g_lin.data_idx = 0;

        if (g_lin.data_len == 0) {
            g_lin.state = LIN_STATE_IDLE;
        }
        else if (b == LIN_PID_BTN_REPORT) {
            lin_slave_send_btn_response();
            g_lin.state = LIN_STATE_IDLE;
        }
        else if (b == LIN_PID_STATUS) {
            lin_slave_send_status_response();
            g_lin.state = LIN_STATE_IDLE;
        }
        else {
            g_lin.state = LIN_STATE_DATA;
        }
        break;

    case LIN_STATE_DATA:
        g_lin.data[g_lin.data_idx++] = b;
        if (g_lin.data_idx >= g_lin.data_len) {
            g_lin.state = LIN_STATE_CHECKSUM;
        }
        break;

    case LIN_STATE_CHECKSUM:
        g_lin.checksum = b;
        lin_handle_frame();
        g_lin.state = LIN_STATE_IDLE;
        break;
    }
}

void LIN_Slave_MainFunction(uint32_t now_ms)
{
    uint32_t primask;

    if (LinNmSlave_IsProtocolEnabled() &&
        (g_lin.state != LIN_STATE_IDLE) &&
        ((uint32_t)(now_ms - g_lin.last_activity_tick) >=
         LIN_FRAME_TIMEOUT_MS)) {
        primask = __get_PRIMASK();
        __disable_irq();
        if ((g_lin.state != LIN_STATE_IDLE) &&
            ((uint32_t)(now_ms - g_lin.last_activity_tick) >=
             LIN_FRAME_TIMEOUT_MS)) {
            g_lin.err_cnt++;
            g_lin.err_timeout++;
            lin_slave_recover();
        }
        if (primask == 0U) {
            __enable_irq();
        }
    }

    if (LinNmSlave_IsNetworkAwake() &&
        ((uint32_t)(now_ms - s_diag_log_tick) >= LIN_DIAG_LOG_PERIOD_MS)) {
        s_diag_log_tick = now_ms;
        printf("[LIN] NM=%u SLP=%u UART=%u parser=%u arm=%lu "
               "RX21=%lu TX22=%lu TX23=%lu pending=%u ack=%lu drop=%lu "
               "lock=%u reject=%lu anti=%u "
               "sup=%u ERR sync=%lu pid=%lu cs=%lu to=%lu uart_real=%lu recovery_real=%lu "
               "CR1=%04lX CR2=%04lX SR=%04lX\r\n",
               (unsigned int)LinNmSlave_GetState(),
               (unsigned int)C8_LinPhy_IsNormal(),
               (unsigned int)LIN_Slave_IsUartReady(),
               (unsigned int)g_lin.state,
               (unsigned long)s_transport_arm_count,
               (unsigned long)g_lin.rx_21_cnt,
               (unsigned long)g_lin.tx_22_cnt,
               (unsigned long)g_lin.tx_23_cnt,
               (unsigned int)s_button_count,
               (unsigned long)s_button_ack_count,
               (unsigned long)s_button_drop_count,
               (unsigned int)(s_window_switch_enabled == 0U),
               (unsigned long)s_lock_reject_count,
               (unsigned int)s_antipinch_warning,
               (unsigned int)LinNmSlave_IsGestureSuppressed(),
               (unsigned long)g_lin.err_sync,
               (unsigned long)g_lin.err_pid,
               (unsigned long)g_lin.err_chk,
               (unsigned long)g_lin.err_timeout,
               (unsigned long)g_lin.err_uart,
               (unsigned long)g_lin.recovery_cnt,
               (unsigned long)huart3.Instance->CR1,
               (unsigned long)huart3.Instance->CR2,
               (unsigned long)huart3.Instance->SR);
    }
}
