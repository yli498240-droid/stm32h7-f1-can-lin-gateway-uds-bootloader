#ifndef CAN_TP_H
#define CAN_TP_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define CANTP_TX_CAN_ID          0x7E8U
#define CANTP_RX_CAN_ID          0x7E0U
#define CANTP_MAX_PAYLOAD        4095U
#define CANTP_RX_TIMEOUT_MS      1000U
#define CANTP_TX_FC_TIMEOUT_MS   1000U

#define CANTP_PCI_TYPE_SF        0x00U
#define CANTP_PCI_TYPE_FF        0x10U
#define CANTP_PCI_TYPE_CF        0x20U
#define CANTP_PCI_TYPE_FC        0x30U
#define CANTP_PCI_TYPE_MASK      0xF0U

typedef enum
{
    CANTP_OK = 0,
    CANTP_INVALID_ARG,
    CANTP_BUSY,
    CANTP_TIMEOUT,
    CANTP_TX_ERROR,
    CANTP_PROTOCOL_ERROR
} CanTp_StatusType;

typedef void (*CanTp_RxCallbackType)(const uint8_t *data, uint16_t length);

void CanTp_Init(void);
/* For multi-frame payloads, CANTP_OK means the asynchronous TX was accepted. */
CanTp_StatusType CanTp_Transmit(const uint8_t *payload, uint16_t length);
void CanTp_RegisterRxCallback(CanTp_RxCallbackType callback);
void CanTp_RxIndication(uint32_t can_id, const uint8_t *data, uint8_t dlc);
void CanTp_MainFunction(uint32_t now_ms);

#endif /* CAN_TP_H */
