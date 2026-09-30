#include "lin_master.h"
#include "usart.h"
#include "gpio.h"
#include "button_event.h"
#include "window_status.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

/* 0x22 诊断仅做低频汇总，避免 printf 干扰 19200 bit/s response 时序。 */
static uint32_t s_lin22_echo_discarded_cnt = 0U;
static uint32_t s_lin22_valid_response_cnt = 0U;
static uint32_t s_lin22_checksum_error_cnt = 0U;
static uint32_t s_lin22_timeout_cnt = 0U;
static uint32_t s_lin22_rx_error_cnt = 0U;
static uint32_t s_lin22_header_error_cnt = 0U;
static uint32_t s_lin22_event_accept_cnt = 0U;
static uint32_t s_lin22_idle_response_cnt = 0U;
static uint32_t s_lin22_invalid_event_cnt = 0U;
static uint32_t s_lin22_diag_tick = 0U;
static uint8_t s_lin22_last_event = 0U;
static uint8_t s_lin22_last_sequence = 0U;

static uint32_t s_lin23_echo_discarded_cnt = 0U;
static uint32_t s_lin23_valid_response_cnt = 0U;
static uint32_t s_lin23_checksum_error_cnt = 0U;
static uint32_t s_lin23_timeout_cnt = 0U;
static uint32_t s_lin23_rx_error_cnt = 0U;
static uint32_t s_lin23_header_error_cnt = 0U;
static uint8_t s_lin23_last_status[4] = {0U};

static uint8_t s_lin21_last_control = 0U;
static uint8_t s_lin21_last_ui = 0U;
static uint32_t s_lin_header_tx_ok_cnt = 0U;
static uint32_t s_lin_header_tx_error_cnt = 0U;
static uint32_t s_lin_response_rx_byte_cnt = 0U;
static uint32_t s_lin_transport_clean_arm_cnt = 0U;
static uint32_t s_lin_last_hal_status = HAL_OK;
static uint8_t s_lin_last_raw[6] = {0U};
static uint8_t s_lin_last_raw_len = 0U;
/* KEY1 task writes, LIN scheduler task reads; uint8_t access is atomic on H7. */
static volatile uint8_t s_window_switch_enabled = 1U;

/* LIN PID 计算:在 ID 上面加 2-bit 奇偶校验 */
static uint8_t lin_calc_pid(uint8_t id)
{
    id &= 0x3F;  /* ID 只占 6-bit */

    uint8_t p0 = ((id >> 0) & 1) ^ ((id >> 1) & 1) ^
                 ((id >> 2) & 1) ^ ((id >> 4) & 1);
    uint8_t p1 = ~(((id >> 1) & 1) ^ ((id >> 3) & 1) ^
                   ((id >> 4) & 1) ^ ((id >> 5) & 1)) & 1;

    return id | (p0 << 6) | (p1 << 7);
}

/* LIN Enhanced Checksum:对 PID + Data 求和的反码 */
static uint8_t lin_calc_checksum(uint8_t pid, uint8_t *data, uint8_t len)
{
    uint16_t sum = pid;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
        if (sum >= 256) sum -= 255;  /* carry wraparound */
    }
    return (uint8_t)(~sum);
}

/* 0x3C/0x3D 诊断帧使用 Classic Checksum，不包含 PID。 */
static uint8_t lin_calc_classic_checksum(const uint8_t *data, uint8_t len)
{
    uint16_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < len; i++) {
        sum += data[i];
        if (sum >= 256U) sum -= 255U;
    }
    return (uint8_t)(~sum);
}

/* RE 位改变后必须等待 REACK，才能把 Header TX 与 Response RX 明确分段。 */
static HAL_StatusTypeDef lin_set_receiver(uint8_t enable)
{
    uint32_t start = HAL_GetTick();

    if (enable != 0U) {
        huart3.Instance->CR1 |= USART_CR1_RE;
        while ((huart3.Instance->ISR & USART_ISR_REACK) == 0U) {
            if ((HAL_GetTick() - start) >= 2U) {
                return HAL_TIMEOUT;
            }
        }
    } else {
        huart3.Instance->CR1 &= ~USART_CR1_RE;
        while ((huart3.Instance->ISR & USART_ISR_REACK) != 0U) {
            if ((HAL_GetTick() - start) >= 2U) {
                return HAL_TIMEOUT;
            }
        }
    }
    return HAL_OK;
}

/* 仅在新 Header 尚未发送、Slave 尚不可能响应时清理前序本机 TX echo。 */
static void lin_flush_receiver_before_header(void)
{
    huart3.Instance->RQR = USART_RQR_RXFRQ;
    huart3.Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF |
                           USART_ICR_NECF  | USART_ICR_PECF |
                           USART_ICR_IDLECF | USART_ICR_LBDCF;
}

static HAL_StatusTypeDef lin_transport_clean_arm(void)
{
    HAL_StatusTypeDef status;

    /* H7 may be reset/reflashed while the physical LIN bus is active. Build a
       deterministic cold-start boundary instead of inheriting RXNE/error/HAL
       state accumulated between MX_USART3 init and the LIN task start. */
    status = HAL_UART_Abort(&huart3);
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RE);
    huart3.Instance->RQR = USART_RQR_RXFRQ;
    huart3.Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF |
                           USART_ICR_NECF  | USART_ICR_PECF |
                           USART_ICR_IDLECF | USART_ICR_LBDCF |
                           USART_ICR_TCCF;
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);

    SET_BIT(huart3.Instance->CR2, USART_CR2_LINEN | USART_CR2_LBDL);
    SET_BIT(huart3.Instance->CR1, USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    if (lin_set_receiver(1U) != HAL_OK) status = HAL_TIMEOUT;
    s_lin_transport_clean_arm_cnt++;
    s_lin_last_hal_status = (uint32_t)status;
    return status;
}

static void lin_diag_log(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_lin22_diag_tick) < 1000U) {
        return;
    }
    s_lin22_diag_tick = now;
    printf("[LIN] 21 ctrl=%02X ui=%02X | 22 valid=%lu event=%lu idle=%lu "
           "duplicate=%lu drop=%lu invalid=%lu cs=%lu to=%lu | "
           "23 valid=%lu btn=%02X fifo=%02X health=%02X alive=%u cs=%lu to=%lu\r\n",
           s_lin21_last_control,
           s_lin21_last_ui,
           (unsigned long)s_lin22_valid_response_cnt,
           (unsigned long)s_lin22_event_accept_cnt,
           (unsigned long)s_lin22_idle_response_cnt,
           (unsigned long)ButtonEvent_GetDuplicateCount(),
           (unsigned long)ButtonEvent_GetQueueDropCount(),
           (unsigned long)s_lin22_invalid_event_cnt,
           (unsigned long)s_lin22_checksum_error_cnt,
           (unsigned long)s_lin22_timeout_cnt,
           (unsigned long)s_lin23_valid_response_cnt,
           s_lin23_last_status[0],
           s_lin23_last_status[1],
           s_lin23_last_status[2],
           (unsigned int)s_lin23_last_status[3],
           (unsigned long)s_lin23_checksum_error_cnt,
           (unsigned long)s_lin23_timeout_cnt);
    if ((s_lin22_valid_response_cnt == 0U) ||
        (s_lin23_valid_response_cnt == 0U)) {
        printf("[LIN-PHY] hdr_ok/err=%lu/%lu rx_bytes=%lu clean=%lu hal=%lu "
               "g/rx=%u/%u err=0x%08lX CR1=%08lX CR2=%08lX ISR=%08lX "
               "BRR=%08lX SLP=%u raw=%02X/%02X/%02X len=%u\r\n",
               (unsigned long)s_lin_header_tx_ok_cnt,
               (unsigned long)s_lin_header_tx_error_cnt,
               (unsigned long)s_lin_response_rx_byte_cnt,
               (unsigned long)s_lin_transport_clean_arm_cnt,
               (unsigned long)s_lin_last_hal_status,
               (unsigned int)huart3.gState,
               (unsigned int)huart3.RxState,
               (unsigned long)huart3.ErrorCode,
               (unsigned long)huart3.Instance->CR1,
               (unsigned long)huart3.Instance->CR2,
               (unsigned long)huart3.Instance->ISR,
               (unsigned long)huart3.Instance->BRR,
               (unsigned int)HAL_GPIO_ReadPin(LIN_SLP_H7_GPIO_Port,
                                              LIN_SLP_H7_Pin),
               s_lin_last_raw[0], s_lin_last_raw[1], s_lin_last_raw[2],
               (unsigned int)s_lin_last_raw_len);
    }
}

void lin_master_init(void)
{
    HAL_StatusTypeDef status = lin_transport_clean_arm();

    printf("[LIN] Master Init %s (USART3 19200bps clean-arm=%lu)\r\n",
           (status == HAL_OK) ? "OK" : "ERROR",
           (unsigned long)s_lin_transport_clean_arm_cnt);
}
static HAL_StatusTypeDef lin_master_send_frame_checked(uint8_t id,
                                                        uint8_t *data,
                                                        uint8_t len,
                                                        uint8_t classic)
{
    HAL_StatusTypeDef st;
    uint8_t sync = 0x55U;
    uint8_t pid;
    uint8_t checksum;

    if (len > 8U) return HAL_ERROR;

    pid = lin_calc_pid(id);
    checksum = (classic != 0U) ? lin_calc_classic_checksum(data, len) :
                                lin_calc_checksum(pid, data, len);

    /* Publisher 完整帧不需要接收本机 TJA1021 回声。 */
    st = lin_set_receiver(0U);
    if (st == HAL_OK) lin_flush_receiver_before_header();
    if (st == HAL_OK) st = HAL_LIN_SendBreak(&huart3);
    if (st == HAL_OK) st = HAL_UART_Transmit(&huart3, &sync, 1U, 100U);
    if (st == HAL_OK) st = HAL_UART_Transmit(&huart3, &pid, 1U, 100U);
    if ((st == HAL_OK) && (len > 0U)) {
        st = HAL_UART_Transmit(&huart3, data, len, 100U);
    }
    if (st == HAL_OK) {
        st = HAL_UART_Transmit(&huart3, &checksum, 1U, 100U);
    }

    /* 帧已由 TC 完整发送后才重新打开接收，不把回声留给下一调度槽。 */
    if (lin_set_receiver(1U) != HAL_OK) st = HAL_ERROR;
    return st;
}

void lin_master_send_frame(uint8_t id, uint8_t *data, uint8_t len)
{
    (void)lin_master_send_frame_checked(id, data, len, 0U);
}

/* === Phase 8.3:发 Header(Break+Sync+PID)+ 等从节点 response =====
 * 用于 slave-publisher 帧 0x22/0x23。
 * Break/Sync 发送期间关闭 RX，发送 PID 前打开 RX。TJA1021 可能回送
 * Header 的 55/E2、仅 E2，或不产生可见回声。接收状态机只消费与当前
 * Header 完整匹配的可选前缀，其后字节才进入 Slave Response parser。
 */
void lin_master_send_header_and_recv(uint8_t id, uint8_t expected_data_len)
{
    uint8_t pid = lin_calc_pid(id);
    uint8_t rx_buf[9] = {0};
    uint8_t rx_byte = 0U;
    uint8_t rx_count = 0U;
    uint8_t first_byte_checked = 0U;
    uint8_t total;
    uint32_t rx_start;
    HAL_StatusTypeDef st;

    if (expected_data_len > 8) expected_data_len = 8;
    s_lin_last_raw_len = 0U;

    st = lin_set_receiver(0U);
    lin_flush_receiver_before_header();

    if (st == HAL_OK) {
        st = HAL_LIN_SendBreak(&huart3);
    }
    if (st == HAL_OK) {
        uint8_t sync = 0x55;
        st = HAL_UART_Transmit(&huart3, &sync, 1, 5);
    }
    if (st == HAL_OK) {
        st = lin_set_receiver(1U);
    }
    if (st == HAL_OK) {
        st = HAL_UART_Transmit(&huart3, &pid, 1, 5);
    }

    /* HAL_UART_Transmit 已等待 TC；立即进入 response 接收，不做固定延时。 */
    if (st != HAL_OK) {
        s_lin_header_tx_error_cnt++;
        s_lin_last_hal_status = (uint32_t)st;
        if (id == 0x22U) {
            s_lin22_header_error_cnt++;
            lin_diag_log();
        } else if (id == 0x23U) {
            s_lin23_header_error_cnt++;
        }
        return;
    }
    s_lin_header_tx_ok_cnt++;

    total = expected_data_len + 1U;
    rx_start = HAL_GetTick();
    st = HAL_OK;
    while (rx_count < total) {
        uint32_t elapsed = HAL_GetTick() - rx_start;
        uint32_t remaining;

        if (elapsed >= 15U) {
            st = HAL_TIMEOUT;
            break;
        }
        remaining = 15U - elapsed;
        st = HAL_UART_Receive(&huart3, &rx_byte, 1U, remaining);
        if (st != HAL_OK) {
            break;
        }
        s_lin_response_rx_byte_cnt++;
        if (s_lin_last_raw_len < sizeof(s_lin_last_raw)) {
            s_lin_last_raw[s_lin_last_raw_len++] = rx_byte;
        }

        if (first_byte_checked == 0U) {
            if (rx_byte == 0x55U) {
                if (id == 0x22U) s_lin22_echo_discarded_cnt++;
                else if (id == 0x23U) s_lin23_echo_discarded_cnt++;
                continue;
            }
            if (rx_byte == pid) {
                first_byte_checked = 1U;
                if (id == 0x22U) s_lin22_echo_discarded_cnt++;
                else if (id == 0x23U) s_lin23_echo_discarded_cnt++;
                continue;
            }
            /* 55 后未出现当前 PID：该字节不是已知 Header echo，作为 Data。 */
            first_byte_checked = 1U;
        }
        rx_buf[rx_count++] = rx_byte;
    }

    if (st != HAL_OK) {
        s_lin_last_hal_status = (uint32_t)st;
        if (id == 0x22U) {
            if (st == HAL_TIMEOUT) {
                s_lin22_timeout_cnt++;
            } else {
                s_lin22_rx_error_cnt++;
            }
            lin_diag_log();
        } else if (id == 0x23U) {
            if (st == HAL_TIMEOUT) {
                s_lin23_timeout_cnt++;
            } else {
                s_lin23_rx_error_cnt++;
            }
        }
        return;
    }
    s_lin_last_hal_status = HAL_OK;
    
    /* 后面校验和路由不变 */
    uint8_t cs_expected = lin_calc_checksum(pid, rx_buf, expected_data_len);
    uint8_t cs_got = rx_buf[expected_data_len];
    if (cs_expected != cs_got) {
        if (id == 0x22U) {
            s_lin22_checksum_error_cnt++;
            lin_diag_log();
        } else if (id == 0x23U) {
            s_lin23_checksum_error_cnt++;
        }
        return;
    }
    if (id == 0x22U && expected_data_len == 2U) {
        uint8_t previous_ack = ButtonEvent_GetAckSequence();

        s_lin22_last_event = rx_buf[0];
        s_lin22_last_sequence = rx_buf[1];
        s_lin22_valid_response_cnt++;
        if (rx_buf[0] == 0U) {
            s_lin22_idle_response_cnt++;
        } else if (ButtonEvent_OfferFromLin(rx_buf[0], rx_buf[1]) != 0U) {
            if (rx_buf[1] != previous_ack) {
                s_lin22_event_accept_cnt++;
            }
        } else {
            s_lin22_invalid_event_cnt++;
        }
        lin_diag_log();
    } else if (id == 0x23U && expected_data_len == 4U) {
        uint8_t i;

        for (i = 0U; i < 4U; i++) {
            s_lin23_last_status[i] = rx_buf[i];
        }
        s_lin23_valid_response_cnt++;
    }
}

/* === Phase 7.2 LIN Schedule Table === */

static uint32_t sched_tick = 0;
static uint8_t  sched_last_ack_sent = 0xFFU;
static uint8_t  sched_last_control_sent = 0xFFU;
static uint8_t  sched_last_ui_sent = 0xFFU;

static void lin_master_build_control(uint8_t data[3])
{
    WindowStatus_SnapshotType window;
    uint8_t control = 0U;
    uint8_t ui = 0U;

    WindowStatus_GetSnapshot(HAL_GetTick(), &window);
    if (s_window_switch_enabled != 0U) control |= (1U << 0);
    if (window.antipinch_active != 0U) control |= (1U << 1);
    if (window.window_moving != 0U) control |= (1U << 2);
    if (window.closing != 0U) control |= (1U << 3);
    if (window.rc_online != 0U) ui |= (1U << 0);
    if (window.antipinch_latched != 0U) ui |= (1U << 1);
    if (window.can_healthy != 0U) ui |= (1U << 2);

    data[0] = control;
    data[1] = ui;
    data[2] = ButtonEvent_GetAckSequence();
}

void lin_master_set_window_switch_enable(uint8_t enabled)
{
    s_window_switch_enabled = (enabled != 0U) ? 1U : 0U;
}

/* 20 ms 基准调度：0x22 优先；0x21 保留 100 ms 心跳并即时携带新 ACK；
 * 0x23 保留 200 ms。所有发送均在同一任务中同步完成，不会重入。 */
void lin_master_schedule_tick(void)
{
    uint8_t data[3];
    uint8_t control;
    uint8_t ui;
    uint8_t ack;

    lin_master_build_control(data);
    control = data[0];
    ui = data[1];
    ack = data[2];

    /* 0x21：100 ms 心跳；状态或 ACK 改变时立即更新。Byte2 保留原 ACK。 */
    if (((sched_tick % 5U) == 0U) ||
        (ack != sched_last_ack_sent) ||
        (control != sched_last_control_sent) ||
        (ui != sched_last_ui_sent)) {
        lin_master_send_frame(0x21U, data, 3U);
        s_lin21_last_control = control;
        s_lin21_last_ui = ui;
        sched_last_ack_sent = ack;
        sched_last_control_sent = control;
        sched_last_ui_sent = ui;
    }

    /* 0x22：每个 20 ms tick 都轮询，保持 Header-only 与既有 turnaround。 */
    lin_master_send_header_and_recv(0x22U, 2U);

    /* 0x23：200 ms，H7 只发 Header，C8 发布 4-byte panel status。 */
    if ((sched_tick % 10U) == 2U) {
        lin_master_send_header_and_recv(0x23U, 4U);
    }

    sched_tick++;
}

bool lin_master_is_c8_fifo_empty(void)
{
    return ((s_lin23_valid_response_cnt != 0U) &&
            ((s_lin23_last_status[0] & 0x07U) == 0U) &&
            ((s_lin23_last_status[1] & 0x0FU) == 0U));
}

bool lin_master_confirm_sleep_boundary(void)
{
    uint32_t previous_valid = s_lin23_valid_response_cnt;

    /* GTS 前专门读一次最新 0x23，不使用最多 200 ms 的旧快照。 */
    lin_master_send_header_and_recv(0x23U, 4U);
    return ((s_lin23_valid_response_cnt != previous_valid) &&
            lin_master_is_c8_fifo_empty());
}

bool lin_master_send_go_to_sleep(void)
{
    static uint8_t sleep_data[8] = {
        0x00U, 0xFFU, 0xFFU, 0xFFU,
        0xFFU, 0xFFU, 0xFFU, 0xFFU
    };

    /* ID 0x3C 的线上 PID 仍为 0x3C；该 payload 的 Classic CS=0x00。 */
    return lin_master_send_frame_checked(0x3CU, sleep_data, 8U, 1U) == HAL_OK;
}

bool lin_master_send_wake_sync(void)
{
    uint8_t data[3];

    lin_master_build_control(data);
    if (lin_master_send_frame_checked(0x21U, data, 3U, 0U) != HAL_OK) {
        return false;
    }

    s_lin21_last_control = data[0];
    s_lin21_last_ui = data[1];
    sched_last_control_sent = data[0];
    sched_last_ui_sent = data[1];
    sched_last_ack_sent = data[2];
    /* 下一次 AWAKE tick 从 0x22 轮询继续，不追赶 Sleep 期间节拍。 */
    sched_tick = 1U;
    return true;
}

bool lin_master_send_diag_request(const uint8_t request[8])
{
    return (request != NULL) &&
           (lin_master_send_frame_checked(
                0x3CU, (uint8_t *)request, 8U, 1U) == HAL_OK);
}

bool lin_master_receive_diag_response(uint8_t response[8])
{
    const uint8_t pid = lin_calc_pid(0x3DU);
    uint8_t rx_buf[9] = {0U};
    uint8_t rx_byte = 0U;
    uint8_t rx_count = 0U;
    uint8_t first_byte_checked = 0U;
    uint8_t sync = 0x55U;
    uint32_t rx_start;
    HAL_StatusTypeDef st;

    if (response == NULL) return false;
    st = lin_set_receiver(0U);
    lin_flush_receiver_before_header();
    if (st == HAL_OK) st = HAL_LIN_SendBreak(&huart3);
    if (st == HAL_OK) st = HAL_UART_Transmit(&huart3, &sync, 1U, 5U);
    if (st == HAL_OK) st = lin_set_receiver(1U);
    if (st == HAL_OK) st = HAL_UART_Transmit(&huart3, (uint8_t *)&pid, 1U, 5U);
    if (st != HAL_OK) return false;

    rx_start = HAL_GetTick();
    while (rx_count < 9U) {
        uint32_t elapsed = HAL_GetTick() - rx_start;
        uint32_t remaining;

        if (elapsed >= 15U) return false;
        remaining = 15U - elapsed;
        st = HAL_UART_Receive(&huart3, &rx_byte, 1U, remaining);
        if (st != HAL_OK) return false;

        /* Preserve the already Hardware-Gated TJA1021 header-echo handling. */
        if (first_byte_checked == 0U) {
            if (rx_byte == 0x55U) continue;
            if (rx_byte == pid) {
                first_byte_checked = 1U;
                continue;
            }
            first_byte_checked = 1U;
        }
        rx_buf[rx_count++] = rx_byte;
    }

    if (lin_calc_classic_checksum(rx_buf, 8U) != rx_buf[8]) return false;
    for (rx_count = 0U; rx_count < 8U; rx_count++) {
        response[rx_count] = rx_buf[rx_count];
    }
    return true;
}
