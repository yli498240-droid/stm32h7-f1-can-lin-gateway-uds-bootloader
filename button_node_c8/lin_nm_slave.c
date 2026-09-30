#include "lin_nm_slave.h"

#include "button.h"
#include "lin_phy.h"
#include "lin_slave.h"
#include <stdio.h>

#define C8_LIN_WAKE_PHY_SETTLE_MS       1U
#define C8_LIN_WAKE_RETRY_MS          250U
#define C8_LIN_WAKE_RETRY_PAUSE_MS   1500U
#define C8_LIN_WAKE_RETRY_COUNT          3U

static volatile C8_LinNm_StateType s_state = C8_LIN_NM_AWAKE;
static volatile uint8_t s_suppress_gesture = 0U;
static volatile uint8_t s_wake_complete = 0U;
static volatile uint8_t s_wake_sync_log_pending = 0U;
static uint8_t s_wake_attempt = 0U;
static uint32_t s_next_wake_pulse_ms = 0U;

void LinNmSlave_Init(void)
{
    C8_LinPhy_Init();
    s_state = C8_LIN_NM_AWAKE;
    s_suppress_gesture = 0U;
    s_wake_complete = 0U;
    s_wake_sync_log_pending = 0U;
    s_wake_attempt = 0U;
    s_next_wake_pulse_ms = 0U;
    printf("[LIN-NM] cold boot -> AWAKE, SLP_N=HIGH\r\n");
}

void LinNmSlave_NotifyGoToSleepFromIsr(void)
{
    if (s_state == C8_LIN_NM_AWAKE) {
        s_state = C8_LIN_NM_GOING_TO_SLEEP;
    }
}

void LinNmSlave_NotifyValidControlFromIsr(void)
{
    if (s_state == C8_LIN_NM_WAKEUP_PENDING) {
        s_state = C8_LIN_NM_AWAKE;
        s_wake_complete = 1U;
        s_wake_sync_log_pending = 1U;
    }
}

bool LinNmSlave_IsProtocolEnabled(void)
{
    return ((s_state == C8_LIN_NM_AWAKE) ||
            (s_state == C8_LIN_NM_WAKEUP_PENDING));
}

bool LinNmSlave_IsNetworkAwake(void)
{
    return s_state == C8_LIN_NM_AWAKE;
}

bool LinNmSlave_ShouldSuppressButtonEvents(void)
{
    return (s_suppress_gesture != 0U) ||
           (s_state != C8_LIN_NM_AWAKE);
}

bool LinNmSlave_IsGestureSuppressed(void)
{
    return s_suppress_gesture != 0U;
}

bool LinNmSlave_TakeWakeCompleted(void)
{
    bool complete = s_wake_complete != 0U;
    s_wake_complete = 0U;
    return complete;
}

C8_LinNm_StateType LinNmSlave_GetState(void)
{
    return s_state;
}

void LinNmSlave_MainFunction(uint32_t now_ms, uint8_t physical_buttons)
{
    /* 释放条件与 NM state 解耦：首帧 0x21 可能在首次手势
     * 释放前就把 state 切到 AWAKE，但释放后仍必须解除 suppress。 */
    if ((s_suppress_gesture != 0U) &&
        ((physical_buttons & 0x03U) == 0U)) {
        Button_ClearEvents();
        s_suppress_gesture = 0U;
        printf("[LIN-NM] button suppression released\r\n");
    }

    if (s_state == C8_LIN_NM_GOING_TO_SLEEP) {
        printf("[LIN-NM] Go-To-Sleep received\r\n");
        LIN_Slave_ClearButtonEvents();
        Button_ClearEvents();
        s_suppress_gesture = 0U;
        LIN_Slave_SuspendTransport();
        C8_LinPhy_SetSleep();
        s_state = C8_LIN_NM_SLEEP;
        printf("[LIN-NM] LIN entered Sleep; SLP_N=LOW, LEDs OFF\r\n");
        return;
    }

    if (s_state == C8_LIN_NM_SLEEP) {
        if ((physical_buttons & 0x03U) != 0U) {
            /* 首个完整手势只唤醒，直到释放都禁止入 FIFO。 */
            s_suppress_gesture = 1U;
            Button_ClearEvents();
            LIN_Slave_ClearButtonEvents();
            C8_LinPhy_SetNormal();
            s_wake_attempt = 0U;
            s_next_wake_pulse_ms = now_ms + C8_LIN_WAKE_PHY_SETTLE_MS;
            s_state = C8_LIN_NM_WAKEUP_PENDING;
            printf("[LIN-NM] Local button Wake; first gesture suppressed\r\n");
        }
        return;
    }

    if (s_state == C8_LIN_NM_WAKEUP_PENDING) {
        if (s_suppress_gesture != 0U) {
            Button_ClearEvents();
        }

        if ((int32_t)(now_ms - s_next_wake_pulse_ms) >= 0) {
            bool pulse_ok = C8_LinPhy_SendWakePulse();

            /* Wake pulse 边界后回到 M12.6 的同一直接 IRQ/parser 入口。 */
            LIN_Slave_ArmAwakeTransport();
            printf("[LIN-NM] USART3 restored: ready=%u parser=WAIT_BREAK\r\n",
                   (unsigned int)LIN_Slave_IsUartReady());
            if (pulse_ok) {
                s_wake_attempt++;
                printf("[LIN-NM] Wake pulse sent (%u), dominant ~=677 us\r\n",
                       (unsigned int)s_wake_attempt);
            }
            if (s_wake_attempt >= C8_LIN_WAKE_RETRY_COUNT) {
                s_wake_attempt = 0U;
                s_next_wake_pulse_ms = now_ms + C8_LIN_WAKE_RETRY_PAUSE_MS;
            } else {
                s_next_wake_pulse_ms = now_ms + C8_LIN_WAKE_RETRY_MS;
            }
        }
        return;
    }

    if (s_wake_sync_log_pending != 0U) {
        s_wake_sync_log_pending = 0U;
        printf("[LIN-NM] first 0x21 after wake received\r\n");
        printf("[LIN-NM] wake sync complete; publisher enabled\r\n");
    }
}
