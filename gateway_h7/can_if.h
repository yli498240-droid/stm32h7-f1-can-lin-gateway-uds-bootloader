#ifndef CAN_IF_H
#define CAN_IF_H

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

typedef enum
{
    CANIF_OK = 0,
    CANIF_ERROR,
    CANIF_INVALID_ARG,
    CANIF_BUSY
} CanIf_StatusType;

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanIf_PduType;

typedef void (*CanIf_TxResultObserverType)(CanIf_StatusType status);
typedef enum {CANIF_BUS_ONLINE=0,CANIF_BUS_OFF,CANIF_BUS_RECOVERING,CANIF_BUS_FAILED} CanIf_BusStateType;
typedef struct {uint32_t bus_off_count,recovery_attempt_count,recovery_success_count,recovery_fail_count;} CanIf_ReliabilityStatsType;
typedef struct
{
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t last_error_code;
    uint32_t bus_off;
    uint32_t error_passive;
    uint32_t error_warning;
    uint32_t callback_count;
    uint32_t last_callback_its;
    uint32_t callback_bus_off;
    uint32_t callback_tx_error_count;
    uint32_t callback_rx_error_count;
    uint32_t callback_last_error_code;
} CanIf_M13BusDiagnosticType;

#define CANIF_RX_QUEUE_STORAGE_SIZE  16U
#define CANIF_RX_QUEUE_CAPACITY      (CANIF_RX_QUEUE_STORAGE_SIZE - 1U)

void CanIf_Init(FDCAN_HandleTypeDef *hfdcan);
void CanIf_SetOnlineAfterStart(void);
void CanIf_NotifyBusOffFromIsr(void);
void CanIf_RecordErrorStatusFromIsr(uint32_t error_status_its,
                                    uint32_t protocol_status_register,
                                    uint32_t error_counter_register);
void CanIf_MainFunction(uint32_t now_ms);
CanIf_BusStateType CanIf_GetBusState(void);
void CanIf_GetReliabilityStats(CanIf_ReliabilityStatsType *stats);
CanIf_StatusType CanIf_M13StartBusOffGate(void);
void CanIf_M13StopBusOffGate(void);
CanIf_StatusType CanIf_M13StartRcBusOffInjector(void);
void CanIf_M13SetRcBusOffInjectorDominant(bool dominant);
void CanIf_M13StopRcBusOffInjector(void);
bool CanIf_M13GetBusDiagnostic(CanIf_M13BusDiagnosticType *diagnostic);
void CanIf_RegisterTxResultObserver(CanIf_TxResultObserverType observer);
CanIf_StatusType CanIf_Transmit(const CanIf_PduType *pdu);
CanIf_StatusType CanIf_NormalizeRx(const FDCAN_RxHeaderTypeDef *header,
                                    const uint8_t *data,
                                    CanIf_PduType *pdu);
CanIf_StatusType CanIf_RxEnqueueFromIsr(const CanIf_PduType *pdu);
bool CanIf_TryGetRxPdu(CanIf_PduType *pdu);
uint32_t CanIf_GetRxDropCount(void);

#endif /* CAN_IF_H */
