#include "lin_nm.h"

#include "main.h"
#include "button_event.h"
#include "lin_diag_client.h"
#include "lin_master.h"
#include "lin_phy.h"
#include <stdio.h>

#define LIN_NM_SLEEP_DRAIN_TIMEOUT_MS  1500U
#define LIN_NM_WAKE_GUARD_MS            100U

static volatile LinNm_StateType s_state = LIN_NM_AWAKE;
static volatile uint8_t s_sleep_request = 0U;
static uint32_t s_state_tick_ms = 0U;

void LinNm_Init(void)
{
    LinPhy_Init();
    s_state = LIN_NM_AWAKE;
    s_sleep_request = 0U;
    s_state_tick_ms = 0U;
    LinDiagClient_Init(HAL_GetTick());
}

bool LinNm_RequestSleep(void)
{
    if ((s_state != LIN_NM_AWAKE) || (s_sleep_request != 0U)) {
        return false;
    }
    s_sleep_request = 1U;
    return true;
}

LinNm_StateType LinNm_GetState(void)
{
    return s_state;
}

void LinNm_MainFunction(uint32_t now_ms)
{
    switch (s_state) {
        case LIN_NM_AWAKE:
            if (s_sleep_request != 0U) {
                s_sleep_request = 0U;
                LinDiagClient_AbortForSleep();
                s_state_tick_ms = now_ms;
                s_state = LIN_NM_GOING_TO_SLEEP;
                printf("[LIN-NM] LIN Sleep requested; draining ButtonEvent/ACK\r\n");
                lin_master_schedule_tick();
            } else if (!LinDiagClient_MainFunction(now_ms)) {
                /* 诊断时隙与正式 schedule 交织；非诊断 tick 保持原调度。 */
                lin_master_schedule_tick();
            }
            break;

        case LIN_NM_GOING_TO_SLEEP:
            /* 继续 schedule，直到 C8 报告 FIFO=0 且 H7 本地已接收队列为空。 */
            lin_master_schedule_tick();
            if ((ButtonEvent_HasPending() == 0U) &&
                lin_master_confirm_sleep_boundary()) {
                if (lin_master_send_go_to_sleep()) {
                    printf("[LIN-NM] Go-To-Sleep sent: PID=3C data=00/FF.. CS=00\r\n");
                    LinPhy_EnterSleepAndArmWake();
                    s_state = LIN_NM_SLEEP;
                    printf("[LIN-NM] LIN entered Sleep; schedule stopped\r\n");
                }
            } else if ((uint32_t)(now_ms - s_state_tick_ms) >=
                       LIN_NM_SLEEP_DRAIN_TIMEOUT_MS) {
                /* 不丢弃已接受业务事件；无法完成 ACK 边界则取消 Sleep。 */
                s_state = LIN_NM_AWAKE;
                printf("[LIN-NM] Sleep cancelled: ButtonEvent/ACK drain timeout\r\n");
            }
            break;

        case LIN_NM_SLEEP:
            if (LinPhy_TakeRemoteWake()) {
                printf("[LIN-NM] Remote Wake detected on USART3_RX/PB11\r\n");
                LinPhy_SetNormal();
                s_state_tick_ms = now_ms;
                s_state = LIN_NM_WAKEUP_PENDING;
            }
            break;

        case LIN_NM_WAKEUP_PENDING:
            if ((uint32_t)(now_ms - s_state_tick_ms) >=
                LIN_NM_WAKE_GUARD_MS) {
                /* 先用 0x21 恢复 Lock/Window/Antipinch/ACK，再重开完整 schedule。 */
                if (lin_master_send_wake_sync()) {
                    LinDiagClient_NotifyWake(now_ms);
                    s_state = LIN_NM_AWAKE;
                    printf("[LIN-NM] LIN Wake complete; 0x21 sync sent, schedule resumed\r\n");
                }
            }
            break;

        default:
            LinPhy_SetNormal();
            s_state = LIN_NM_AWAKE;
            break;
    }
}
