#include "can_if.h"

#include <string.h>
#include <stdio.h>

static FDCAN_HandleTypeDef *s_hfdcan = NULL;
/* SPSC ring: ISR publishes head; defaultTask publishes tail. One slot stays empty. */
static CanIf_PduType s_rx_queue[CANIF_RX_QUEUE_STORAGE_SIZE];
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;
static volatile uint32_t s_rx_drop_count = 0U;
static CanIf_TxResultObserverType s_tx_result_observer = NULL;
static volatile CanIf_BusStateType s_bus_state=CANIF_BUS_FAILED;static volatile uint8_t s_bus_off_pending=0U;
static uint8_t s_retry_in_round=0U;static uint32_t s_retry_tick=0U;static CanIf_ReliabilityStatsType s_stats;
static volatile uint32_t s_m13_callback_count=0U;
static volatile uint32_t s_m13_last_callback_its=0U;
static volatile uint32_t s_m13_callback_bus_off=0U;
static volatile uint32_t s_m13_callback_tec=0U;
static volatile uint32_t s_m13_callback_rec=0U;
static volatile uint32_t s_m13_callback_lec=0U;
static FDCAN_InitTypeDef s_m13_nominal_init;
static uint8_t s_m13_nominal_init_valid=0U;
static volatile uint8_t s_m13_gate_active=0U;
static volatile uint8_t s_m13_rc_injector_active=0U;
static uint8_t s_restore_nominal_on_recovery=0U;
#define CANIF_RECOVERY_BACKOFF_MS 250U
#define CANIF_FAILED_RETRY_MS 5000U
#define CANIF_MAX_RECOVERY_ATTEMPTS 3U
#define CANIF_M13_FAULT_PRESCALER_MULTIPLIER 2U
#define CANIF_NOTIFICATIONS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF)

static uint8_t CanIf_ConfigureAndStart(const FDCAN_InitTypeDef *configuration)
{FDCAN_FilterTypeDef f={0};f.IdType=FDCAN_STANDARD_ID;f.FilterIndex=0;f.FilterType=FDCAN_FILTER_MASK;f.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;f.FilterID1=0U;f.FilterID2=0U;
 if((s_hfdcan==NULL)||(configuration==NULL))return 0U;(void)HAL_FDCAN_Stop(s_hfdcan);(void)HAL_FDCAN_DeInit(s_hfdcan);s_hfdcan->Init=*configuration;if(HAL_FDCAN_Init(s_hfdcan)!=HAL_OK)return 0U;if(HAL_FDCAN_ConfigFilter(s_hfdcan,&f)!=HAL_OK)return 0U;if(HAL_FDCAN_ConfigGlobalFilter(s_hfdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE)!=HAL_OK)return 0U;if(HAL_FDCAN_Start(s_hfdcan)!=HAL_OK)return 0U;if(HAL_FDCAN_ActivateNotification(s_hfdcan,CANIF_NOTIFICATIONS,0U)!=HAL_OK)return 0U;return 1U;}

static uint8_t CanIf_Recover(void)
{
    FDCAN_InitTypeDef configuration;
    if(s_hfdcan==NULL)return 0U;
    if((s_restore_nominal_on_recovery!=0U)&&(s_m13_nominal_init_valid!=0U)){
        configuration=s_m13_nominal_init;
    }else{
        configuration=s_hfdcan->Init;
    }
    if(CanIf_ConfigureAndStart(&configuration)==0U)return 0U;
    s_restore_nominal_on_recovery=0U;
    return 1U;
}

static CanIf_StatusType CanIf_NotifyTxResult(CanIf_StatusType status)
{
    if (s_tx_result_observer != NULL) {
        s_tx_result_observer(status);
    }

    return status;
}

static CanIf_StatusType CanIf_FdcanDlcToBytes(uint32_t data_length,
                                               uint8_t *dlc)
{
    if (dlc == NULL) {
        return CANIF_INVALID_ARG;
    }

    switch (data_length) {
        case FDCAN_DLC_BYTES_0: *dlc = 0U; break;
        case FDCAN_DLC_BYTES_1: *dlc = 1U; break;
        case FDCAN_DLC_BYTES_2: *dlc = 2U; break;
        case FDCAN_DLC_BYTES_3: *dlc = 3U; break;
        case FDCAN_DLC_BYTES_4: *dlc = 4U; break;
        case FDCAN_DLC_BYTES_5: *dlc = 5U; break;
        case FDCAN_DLC_BYTES_6: *dlc = 6U; break;
        case FDCAN_DLC_BYTES_7: *dlc = 7U; break;
        case FDCAN_DLC_BYTES_8: *dlc = 8U; break;
        default: return CANIF_INVALID_ARG;
    }

    return CANIF_OK;
}

static CanIf_StatusType CanIf_BytesToFdcanDlc(uint8_t dlc,
                                               uint32_t *data_length)
{
    if (data_length == NULL) {
        return CANIF_INVALID_ARG;
    }

    switch (dlc) {
        case 0U: *data_length = FDCAN_DLC_BYTES_0; break;
        case 1U: *data_length = FDCAN_DLC_BYTES_1; break;
        case 2U: *data_length = FDCAN_DLC_BYTES_2; break;
        case 3U: *data_length = FDCAN_DLC_BYTES_3; break;
        case 4U: *data_length = FDCAN_DLC_BYTES_4; break;
        case 5U: *data_length = FDCAN_DLC_BYTES_5; break;
        case 6U: *data_length = FDCAN_DLC_BYTES_6; break;
        case 7U: *data_length = FDCAN_DLC_BYTES_7; break;
        case 8U: *data_length = FDCAN_DLC_BYTES_8; break;
        default: return CANIF_INVALID_ARG;
    }

    return CANIF_OK;
}

void CanIf_Init(FDCAN_HandleTypeDef *hfdcan)
{
    s_hfdcan = hfdcan;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_drop_count = 0U;
    s_tx_result_observer = NULL;
    memset(&s_stats,0,sizeof(s_stats));s_bus_state=CANIF_BUS_FAILED;s_bus_off_pending=0U;s_retry_in_round=0U;s_retry_tick=0U;
    s_m13_callback_count=0U;s_m13_last_callback_its=0U;s_m13_callback_bus_off=0U;
    s_m13_callback_tec=0U;s_m13_callback_rec=0U;s_m13_callback_lec=0U;
    s_m13_nominal_init_valid=0U;s_m13_gate_active=0U;s_m13_rc_injector_active=0U;s_restore_nominal_on_recovery=0U;
}
void CanIf_SetOnlineAfterStart(void){s_bus_state=CANIF_BUS_ONLINE;}
void CanIf_NotifyBusOffFromIsr(void)
{
    s_stats.bus_off_count++;s_bus_off_pending=1U;
}
void CanIf_RecordErrorStatusFromIsr(uint32_t error_status_its,
                                    uint32_t protocol_status_register,
                                    uint32_t error_counter_register)
{
    s_m13_callback_count++;
    s_m13_last_callback_its=error_status_its;
    s_m13_callback_lec=(protocol_status_register&FDCAN_PSR_LEC)>>FDCAN_PSR_LEC_Pos;
    s_m13_callback_tec=(error_counter_register&FDCAN_ECR_TEC)>>FDCAN_ECR_TEC_Pos;
    s_m13_callback_rec=(error_counter_register&FDCAN_ECR_REC)>>FDCAN_ECR_REC_Pos;
    if((protocol_status_register&FDCAN_PSR_BO)!=0U){s_m13_callback_bus_off++;}
}
CanIf_BusStateType CanIf_GetBusState(void){return s_bus_state;}
void CanIf_GetReliabilityStats(CanIf_ReliabilityStatsType*s){if(s!=NULL)*s=s_stats;}
CanIf_StatusType CanIf_M13StartBusOffGate(void)
{
    FDCAN_InitTypeDef fault_configuration;

    if((s_hfdcan==NULL)||(s_bus_state!=CANIF_BUS_ONLINE)||(s_m13_gate_active!=0U))return CANIF_BUSY;

    s_m13_nominal_init=s_hfdcan->Init;
    s_m13_nominal_init_valid=1U;
    fault_configuration=s_m13_nominal_init;
    fault_configuration.NominalPrescaler*=CANIF_M13_FAULT_PRESCALER_MULTIPLIER;
    s_bus_state=CANIF_BUS_RECOVERING;
    if(CanIf_ConfigureAndStart(&fault_configuration)==0U){
        s_m13_gate_active=0U;
        if(CanIf_ConfigureAndStart(&s_m13_nominal_init)!=0U)s_bus_state=CANIF_BUS_ONLINE;
        else s_bus_state=CANIF_BUS_FAILED;
        return CANIF_ERROR;
    }
    s_m13_gate_active=1U;
    s_bus_state=CANIF_BUS_ONLINE;
    return CANIF_OK;
}

CanIf_StatusType CanIf_M13StartRcBusOffInjector(void)
{
    if((s_hfdcan==NULL)||(s_bus_state!=CANIF_BUS_ONLINE)||
       (s_m13_gate_active!=0U)||(s_m13_rc_injector_active!=0U))return CANIF_BUSY;

    s_m13_nominal_init=s_hfdcan->Init;
    s_m13_nominal_init_valid=1U;
    s_bus_state=CANIF_BUS_RECOVERING;
    if(HAL_FDCAN_Stop(s_hfdcan)!=HAL_OK){
        if(CanIf_ConfigureAndStart(&s_m13_nominal_init)!=0U)s_bus_state=CANIF_BUS_ONLINE;
        else{s_bus_state=CANIF_BUS_FAILED;s_retry_tick=HAL_GetTick()+CANIF_FAILED_RETRY_MS;}
        return CANIF_ERROR;
    }

    /*
     * M13 RC Gate helper. HAL_FDCAN_Stop leaves INIT and CCE set. Enabling
     * CCCR.TEST allows TEST.TX to control the physical FDCAN_TX pin directly.
     * Start in forced-recessive state; the task later emits bounded dominant
     * pulses. The protocol core is stopped, so H7 cannot become the target
     * transmitter and every normal CanIf_Transmit() remains blocked.
     */
    SET_BIT(s_hfdcan->Instance->CCCR,FDCAN_CCCR_TEST);
    MODIFY_REG(s_hfdcan->Instance->TEST,FDCAN_TEST_TX,
               (3UL<<FDCAN_TEST_TX_Pos));
    __DSB();
    s_m13_rc_injector_active=1U;
    s_bus_state=CANIF_BUS_RECOVERING;
    return CANIF_OK;
}

void CanIf_M13SetRcBusOffInjectorDominant(bool dominant)
{
    uint32_t tx_control;

    if((s_m13_rc_injector_active==0U)||(s_hfdcan==NULL))return;
    /* TEST.TX=10 forces dominant; TEST.TX=11 forces recessive. */
    tx_control=(dominant)?(2UL<<FDCAN_TEST_TX_Pos):
                            (3UL<<FDCAN_TEST_TX_Pos);
    MODIFY_REG(s_hfdcan->Instance->TEST,FDCAN_TEST_TX,tx_control);
    __DSB();
}

void CanIf_M13StopRcBusOffInjector(void)
{
    if((s_m13_rc_injector_active!=0U)&&(s_m13_nominal_init_valid!=0U)&&
       (s_hfdcan!=NULL)){
        MODIFY_REG(s_hfdcan->Instance->TEST,FDCAN_TEST_TX,
                   (3UL<<FDCAN_TEST_TX_Pos));
        __DSB();
        CLEAR_BIT(s_hfdcan->Instance->TEST,FDCAN_TEST_TX);
        CLEAR_BIT(s_hfdcan->Instance->CCCR,FDCAN_CCCR_TEST);
        s_m13_rc_injector_active=0U;
        s_bus_state=CANIF_BUS_RECOVERING;
        if(CanIf_ConfigureAndStart(&s_m13_nominal_init)!=0U){s_bus_state=CANIF_BUS_ONLINE;}
        else{s_bus_state=CANIF_BUS_FAILED;s_retry_tick=HAL_GetTick()+CANIF_FAILED_RETRY_MS;}
    }
}
void CanIf_M13StopBusOffGate(void)
{
    if((s_m13_gate_active!=0U)&&(s_m13_nominal_init_valid!=0U)&&(s_hfdcan!=NULL)){
        s_m13_gate_active=0U;
        s_bus_state=CANIF_BUS_RECOVERING;
        if(CanIf_ConfigureAndStart(&s_m13_nominal_init)!=0U){s_bus_state=CANIF_BUS_ONLINE;}
        else{s_bus_state=CANIF_BUS_FAILED;s_retry_tick=HAL_GetTick()+CANIF_FAILED_RETRY_MS;}
    }
}
bool CanIf_M13GetBusDiagnostic(CanIf_M13BusDiagnosticType *diagnostic)
{
    FDCAN_ErrorCountersTypeDef counters;
    FDCAN_ProtocolStatusTypeDef status;

    if((diagnostic==NULL)||(s_hfdcan==NULL))return false;
    if((HAL_FDCAN_GetErrorCounters(s_hfdcan,&counters)!=HAL_OK)||
       (HAL_FDCAN_GetProtocolStatus(s_hfdcan,&status)!=HAL_OK))return false;
    diagnostic->tx_error_count=counters.TxErrorCnt;
    diagnostic->rx_error_count=counters.RxErrorCnt;
    diagnostic->last_error_code=status.LastErrorCode;
    diagnostic->bus_off=status.BusOff;
    diagnostic->error_passive=status.ErrorPassive;
    diagnostic->error_warning=status.Warning;
    diagnostic->callback_count=s_m13_callback_count;
    diagnostic->last_callback_its=s_m13_last_callback_its;
    diagnostic->callback_bus_off=s_m13_callback_bus_off;
    diagnostic->callback_tx_error_count=s_m13_callback_tec;
    diagnostic->callback_rx_error_count=s_m13_callback_rec;
    diagnostic->callback_last_error_code=s_m13_callback_lec;
    return true;
}
void CanIf_MainFunction(uint32_t now)
{if(s_bus_off_pending!=0U){s_bus_off_pending=0U;if((((s_m13_gate_active!=0U)||(s_m13_rc_injector_active!=0U)))&&(s_m13_nominal_init_valid!=0U)){s_restore_nominal_on_recovery=1U;s_m13_gate_active=0U;s_m13_rc_injector_active=0U;}s_bus_state=CANIF_BUS_OFF;s_retry_tick=now+CANIF_RECOVERY_BACKOFF_MS;s_retry_in_round=0U;printf("[CAN] H7 Bus-Off detected count=%lu\r\n",(unsigned long)s_stats.bus_off_count);}
 if((s_bus_state==CANIF_BUS_OFF||s_bus_state==CANIF_BUS_FAILED)&&((int32_t)(now-s_retry_tick)>=0)){s_bus_state=CANIF_BUS_RECOVERING;s_stats.recovery_attempt_count++;if(CanIf_Recover()){s_bus_state=CANIF_BUS_ONLINE;s_stats.recovery_success_count++;s_retry_in_round=0U;printf("[CAN] H7 recovery success\r\n");}else{s_stats.recovery_fail_count++;s_retry_in_round++;if(s_retry_in_round>=CANIF_MAX_RECOVERY_ATTEMPTS){s_bus_state=CANIF_BUS_FAILED;s_retry_tick=now+CANIF_FAILED_RETRY_MS;s_retry_in_round=0U;printf("[CAN] H7 recovery delayed retry\r\n");}else{s_bus_state=CANIF_BUS_OFF;s_retry_tick=now+CANIF_RECOVERY_BACKOFF_MS;}}}}

void CanIf_RegisterTxResultObserver(CanIf_TxResultObserverType observer)
{
    s_tx_result_observer = observer;
}

CanIf_StatusType CanIf_Transmit(const CanIf_PduType *pdu)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint32_t data_length;

    if ((pdu == NULL) || (pdu->id > 0x7FFU)) {
        return CANIF_INVALID_ARG;
    }
    if (s_hfdcan == NULL) {
        return CanIf_NotifyTxResult(CANIF_ERROR);
    }
    if(s_bus_state!=CANIF_BUS_ONLINE)return CanIf_NotifyTxResult(CANIF_BUSY);
    if (CanIf_BytesToFdcanDlc(pdu->dlc, &data_length) != CANIF_OK) {
        return CANIF_INVALID_ARG;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) == 0U) {
        return CanIf_NotifyTxResult(CANIF_BUSY);
    }

    tx_header.Identifier = pdu->id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = data_length;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan,
                                      &tx_header,
                                      pdu->data) != HAL_OK) {
        return CanIf_NotifyTxResult(CANIF_ERROR);
    }

    return CanIf_NotifyTxResult(CANIF_OK);
}

CanIf_StatusType CanIf_RxEnqueueFromIsr(const CanIf_PduType *pdu)
{
    uint8_t head;
    uint8_t next_head;

    if ((pdu == NULL) || (pdu->id > 0x7FFU) || (pdu->dlc > 8U)) {
        return CANIF_INVALID_ARG;
    }

    head = s_rx_head;
    next_head = (uint8_t)((head + 1U) &
                          (CANIF_RX_QUEUE_STORAGE_SIZE - 1U));
    if (next_head == s_rx_tail) {
        s_rx_drop_count++;
        return CANIF_BUSY;
    }

    s_rx_queue[head] = *pdu;
    /* Publish payload before the producer advances head. */
    __DMB();
    s_rx_head = next_head;
    return CANIF_OK;
}

bool CanIf_TryGetRxPdu(CanIf_PduType *pdu)
{
    uint8_t tail;

    if (pdu == NULL) {
        return false;
    }

    tail = s_rx_tail;
    if (tail == s_rx_head) {
        return false;
    }

    /* Observe the payload after the producer-published head. */
    __DMB();
    *pdu = s_rx_queue[tail];
    /* Finish the copy before releasing this slot back to the ISR producer. */
    __DMB();
    s_rx_tail = (uint8_t)((tail + 1U) &
                          (CANIF_RX_QUEUE_STORAGE_SIZE - 1U));
    return true;
}

uint32_t CanIf_GetRxDropCount(void)
{
    return s_rx_drop_count;
}

CanIf_StatusType CanIf_NormalizeRx(const FDCAN_RxHeaderTypeDef *header,
                                    const uint8_t *data,
                                    CanIf_PduType *pdu)
{
    uint8_t dlc;

    if ((header == NULL) || (data == NULL) || (pdu == NULL)) {
        return CANIF_INVALID_ARG;
    }
    if ((header->Identifier > 0x7FFU) ||
        (header->IdType != FDCAN_STANDARD_ID) ||
        (header->RxFrameType != FDCAN_DATA_FRAME) ||
        (header->FDFormat != FDCAN_CLASSIC_CAN)) {
        return CANIF_INVALID_ARG;
    }
    if (CanIf_FdcanDlcToBytes(header->DataLength, &dlc) != CANIF_OK) {
        return CANIF_INVALID_ARG;
    }

    pdu->id = header->Identifier;
    pdu->dlc = dlc;
    memset(pdu->data, 0, sizeof(pdu->data));
    if (dlc > 0U) {
        memcpy(pdu->data, data, dlc);
    }

    return CANIF_OK;
}
