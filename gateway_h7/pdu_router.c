#include "pdu_router.h"

#include <stddef.h>

#include "can_tp.h"

#define PDUR_H7_WINDOW_STATUS_CAN_ID  0x101U
#define PDUR_H7_DIAG_RESPONSE_CAN_ID  0x7E8U

static PduR_RxHandlerType s_window_status_handler = NULL;

void PduR_Init(PduR_RxHandlerType window_status_handler)
{
    s_window_status_handler = window_status_handler;
}

void PduR_RouteRx(const CanIf_PduType *pdu)
{
    if ((pdu == NULL) || (pdu->id > 0x7FFU) || (pdu->dlc > 8U)) {
        return;
    }

    switch (pdu->id) {
        case PDUR_H7_WINDOW_STATUS_CAN_ID:
            if (s_window_status_handler != NULL) {
                s_window_status_handler(pdu);
            }
            break;

        case PDUR_H7_DIAG_RESPONSE_CAN_ID:
            CanTp_RxIndication(pdu->id, pdu->data, pdu->dlc);
            break;

        default:
            break;
    }
}
