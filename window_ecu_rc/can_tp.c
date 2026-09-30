#include "can_tp.h"

#include "can_if.h"

#include <stdio.h>
#include <string.h>

typedef enum
{
    CANTP_RX_IDLE,
    CANTP_RX_WAIT_CF
} CanTp_RxStateType;

typedef enum
{
    CANTP_TX_IDLE,
    CANTP_TX_WAIT_FC,
    CANTP_TX_SEND_CF
} CanTp_TxStateType;

typedef struct
{
    CanTp_TxStateType state;
    uint16_t total_length;
    uint16_t bytes_sent;
    uint8_t next_sequence_number;
    uint8_t block_size;
    uint8_t block_counter;
    uint8_t st_min_ms;
    uint8_t wait_frame_count;
    uint32_t last_tx_tick_ms;
    uint32_t fc_wait_start_tick_ms;
    uint8_t payload[CANTP_MAX_PAYLOAD];
} CanTp_TxContextType;

#define CANTP_FC_STATUS_CTS       0U
#define CANTP_FC_STATUS_WAIT      1U
#define CANTP_FC_STATUS_OVERFLOW  2U
#define CANTP_TX_WFT_MAX          3U

static CanTp_RxCallbackType s_rx_callback = NULL;
static CanTp_RxStateType s_rx_state = CANTP_RX_IDLE;
static uint8_t s_rx_buffer[CANTP_MAX_PAYLOAD];
static uint16_t s_rx_total_length = 0U;
static uint16_t s_rx_received_length = 0U;
static uint8_t s_rx_expected_sn = 1U;
static uint32_t s_rx_last_activity_ms = 0U;
static bool s_rx_fc_pending = false;
static uint8_t s_rx_fc_frame[8];

static CanTp_TxContextType s_tx_context;

static void CanTp_ResetRxState(void)
{
    s_rx_state = CANTP_RX_IDLE;
    s_rx_total_length = 0U;
    s_rx_received_length = 0U;
    s_rx_expected_sn = 1U;
    s_rx_last_activity_ms = 0U;
}

static void CanTp_ResetTxState(void)
{
    s_tx_context.state = CANTP_TX_IDLE;
    s_tx_context.total_length = 0U;
    s_tx_context.bytes_sent = 0U;
    s_tx_context.next_sequence_number = 1U;
    s_tx_context.block_size = 0U;
    s_tx_context.block_counter = 0U;
    s_tx_context.st_min_ms = 0U;
    s_tx_context.wait_frame_count = 0U;
    s_tx_context.last_tx_tick_ms = 0U;
    s_tx_context.fc_wait_start_tick_ms = 0U;
}

static CanTp_StatusType CanTp_SendCanFrame(const uint8_t *data8)
{
    CanIf_PduType pdu = {0};
    CanIf_StatusType status;

    if (data8 == NULL) {
        return CANTP_INVALID_ARG;
    }

    pdu.id = CANTP_TX_CAN_ID;
    pdu.dlc = 8U;
    memcpy(pdu.data, data8, 8U);

    status = CanIf_Transmit(&pdu);
    if (status == CANIF_BUSY) {
        return CANTP_BUSY;
    }

    if (status != CANIF_OK) {
        return CANTP_TX_ERROR;
    }

    return CANTP_OK;
}

static void CanTp_SendFlowControl(uint8_t fs, uint8_t bs, uint8_t stmin)
{
    uint8_t frame[8] = {0};
    CanTp_StatusType status;

    frame[0] = CANTP_PCI_TYPE_FC | (fs & 0x0FU);
    frame[1] = bs;
    frame[2] = stmin;
    status = CanTp_SendCanFrame(frame);
    if (status == CANTP_BUSY) {
        memcpy(s_rx_fc_frame, frame, sizeof(s_rx_fc_frame));
        s_rx_fc_pending = true;
        printf("[ISOTP TX] FC queued fs=%u bs=%u stmin=%u\r\n",
               fs, bs, stmin);
    } else if (status == CANTP_OK) {
        s_rx_fc_pending = false;
        printf("[ISOTP TX] FC fs=%u bs=%u stmin=%u\r\n", fs, bs, stmin);
    } else {
        printf("[ISOTP TX] FC transmit error\r\n");
    }
}

CanTp_StatusType CanTp_Transmit(const uint8_t *payload, uint16_t length)
{
    uint8_t frame[8] = {0};
    CanTp_StatusType status;

    if ((payload == NULL) || (length == 0U) || (length > CANTP_MAX_PAYLOAD)) {
        return CANTP_INVALID_ARG;
    }

    if (s_tx_context.state != CANTP_TX_IDLE) {
        return CANTP_BUSY;
    }

    if (length <= 7U) {
        frame[0] = CANTP_PCI_TYPE_SF | (uint8_t)length;
        memcpy(&frame[1], payload, length);
        printf("[ISOTP TX] SF len=%u\r\n", length);
        return CanTp_SendCanFrame(frame);
    }

    memcpy(s_tx_context.payload, payload, length);
    s_tx_context.total_length = length;
    s_tx_context.bytes_sent = 6U;
    s_tx_context.next_sequence_number = 1U;
    s_tx_context.block_size = 0U;
    s_tx_context.block_counter = 0U;
    s_tx_context.st_min_ms = 0U;
    s_tx_context.wait_frame_count = 0U;

    frame[0] = CANTP_PCI_TYPE_FF | (uint8_t)((length >> 8) & 0x0FU);
    frame[1] = (uint8_t)(length & 0xFFU);
    memcpy(&frame[2], s_tx_context.payload, 6U);
    printf("[ISOTP TX] FF total_len=%u\r\n", length);
    status = CanTp_SendCanFrame(frame);
    if (status != CANTP_OK) {
        CanTp_ResetTxState();
        return status;
    }

    s_tx_context.fc_wait_start_tick_ms = HAL_GetTick();
    s_tx_context.state = CANTP_TX_WAIT_FC;
    return CANTP_OK;
}

void CanTp_RegisterRxCallback(CanTp_RxCallbackType callback)
{
    s_rx_callback = callback;
}

void CanTp_RxIndication(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t pci_type;

    if ((can_id != CANTP_RX_CAN_ID) || (data == NULL) ||
        (dlc == 0U) || (dlc > 8U)) {
        return;
    }

    pci_type = data[0] & CANTP_PCI_TYPE_MASK;

    switch (pci_type) {
        case CANTP_PCI_TYPE_SF: {
            uint8_t sf_length = data[0] & 0x0FU;

            if ((sf_length == 0U) || (sf_length > 7U) ||
                ((uint8_t)(sf_length + 1U) > dlc)) {
                return;
            }

            printf("[ISOTP RX] SF len=%u, data=", sf_length);
            for (uint8_t i = 0U; i < sf_length; i++) {
                printf("%02X ", data[1U + i]);
            }
            printf("\r\n");

            if (s_rx_callback != NULL) {
                s_rx_callback(&data[1], sf_length);
            }
            CanTp_ResetRxState();
            break;
        }

        case CANTP_PCI_TYPE_FF: {
            uint16_t total_length;

            if (dlc < 8U) {
                return;
            }

            total_length = (uint16_t)((((uint16_t)data[0] & 0x0FU) << 8) |
                                      data[1]);
            if ((total_length <= 7U) || (total_length > CANTP_MAX_PAYLOAD)) {
                printf("[ISOTP RX] FF too long (%u)\r\n", total_length);
                CanTp_ResetRxState();
                return;
            }

            s_rx_total_length = total_length;
            s_rx_received_length = 6U;
            s_rx_expected_sn = 1U;
            memcpy(s_rx_buffer, &data[2], 6U);
            s_rx_last_activity_ms = HAL_GetTick();
            s_rx_state = CANTP_RX_WAIT_CF;

            printf("[ISOTP RX] FF total_len=%u\r\n", total_length);
            CanTp_SendFlowControl(0U, 0U, 0U);
            break;
        }

        case CANTP_PCI_TYPE_CF: {
            uint8_t seq;
            uint16_t remaining;
            uint8_t chunk;

            if (s_rx_state != CANTP_RX_WAIT_CF) {
                printf("[ISOTP RX] CF unexpected\r\n");
                return;
            }
            if (dlc < 2U) {
                CanTp_ResetRxState();
                return;
            }

            seq = data[0] & 0x0FU;
            if (seq != s_rx_expected_sn) {
                printf("[ISOTP RX] CF seq mismatch: got %u, expected %u\r\n",
                       seq, s_rx_expected_sn);
                CanTp_ResetRxState();
                return;
            }

            remaining = (uint16_t)(s_rx_total_length - s_rx_received_length);
            chunk = (remaining > 7U) ? 7U : (uint8_t)remaining;
            if (chunk > (uint8_t)(dlc - 1U)) {
                CanTp_ResetRxState();
                return;
            }

            memcpy(&s_rx_buffer[s_rx_received_length], &data[1], chunk);
            s_rx_received_length = (uint16_t)(s_rx_received_length + chunk);
            s_rx_expected_sn = (uint8_t)((s_rx_expected_sn + 1U) & 0x0FU);
            s_rx_last_activity_ms = HAL_GetTick();

            if (s_rx_received_length >= s_rx_total_length) {
                uint16_t completed_length = s_rx_total_length;

                printf("[ISOTP RX] Multi-frame done, total=%u\r\n",
                       completed_length);
                if (s_rx_callback != NULL) {
                    s_rx_callback(s_rx_buffer, completed_length);
                }
                CanTp_ResetRxState();
            }
            break;
        }

        case CANTP_PCI_TYPE_FC: {
            uint8_t fs;
            uint8_t bs;
            uint8_t stmin;

            if (dlc < 3U) {
                return;
            }

            fs = data[0] & 0x0FU;
            bs = data[1];
            stmin = data[2];
            printf("[ISOTP RX] FC fs=%u bs=%u stmin=%u\r\n", fs, bs, stmin);

            if (s_tx_context.state != CANTP_TX_WAIT_FC) {
                printf("[ISOTP RX] FC unexpected, no TX waiting\r\n");
                break;
            }

            if (fs == CANTP_FC_STATUS_CTS) {
                s_tx_context.block_size = bs;
                s_tx_context.block_counter = 0U;
                /* Preserve the existing simple 0x00..0x7F millisecond support. */
                s_tx_context.st_min_ms = (stmin > 0x7FU) ? 0U : stmin;
                s_tx_context.wait_frame_count = 0U;
                s_tx_context.last_tx_tick_ms = HAL_GetTick();
                s_tx_context.state = CANTP_TX_SEND_CF;
            } else if (fs == CANTP_FC_STATUS_WAIT) {
                s_tx_context.wait_frame_count++;
                if (s_tx_context.wait_frame_count > CANTP_TX_WFT_MAX) {
                    printf("[ISOTP TX] FC WAIT limit exceeded\r\n");
                    CanTp_ResetTxState();
                } else {
                    s_tx_context.fc_wait_start_tick_ms = HAL_GetTick();
                }
            } else if (fs == CANTP_FC_STATUS_OVERFLOW) {
                printf("[ISOTP TX] FC overflow, abort\r\n");
                CanTp_ResetTxState();
            } else {
                printf("[ISOTP TX] FC invalid status, abort\r\n");
                CanTp_ResetTxState();
            }
            break;
        }

        default:
            printf("[ISOTP RX] Unknown PCI 0x%02X\r\n", data[0]);
            break;
    }
}

void CanTp_MainFunction(uint32_t now_ms)
{
    uint8_t frame[8] = {0};
    uint16_t remaining;
    uint8_t chunk;
    CanTp_StatusType status;

    if ((s_rx_state == CANTP_RX_WAIT_CF) &&
        ((uint32_t)(now_ms - s_rx_last_activity_ms) >= CANTP_RX_TIMEOUT_MS)) {
        CanTp_ResetRxState();
        s_rx_fc_pending = false;
        printf("[CANTP] RX timeout, reset reassembly\r\n");
    }

    if (s_rx_fc_pending) {
        status = CanTp_SendCanFrame(s_rx_fc_frame);
        if (status == CANTP_OK) {
            s_rx_fc_pending = false;
            printf("[ISOTP TX] queued FC sent\r\n");
        } else if (status != CANTP_BUSY) {
            s_rx_fc_pending = false;
            printf("[ISOTP TX] queued FC transmit error\r\n");
        }
        return;
    }

    if (s_tx_context.state == CANTP_TX_WAIT_FC) {
        if ((uint32_t)(now_ms - s_tx_context.fc_wait_start_tick_ms) >=
            CANTP_TX_FC_TIMEOUT_MS) {
            printf("[ISOTP TX] FC timeout, abort\r\n");
            CanTp_ResetTxState();
        }
        return;
    }

    if (s_tx_context.state != CANTP_TX_SEND_CF) {
        return;
    }

    if ((uint32_t)(now_ms - s_tx_context.last_tx_tick_ms) <
        (uint32_t)s_tx_context.st_min_ms) {
        return;
    }

    remaining = (uint16_t)(s_tx_context.total_length -
                           s_tx_context.bytes_sent);
    chunk = (remaining > 7U) ? 7U : (uint8_t)remaining;
    frame[0] = CANTP_PCI_TYPE_CF |
               (s_tx_context.next_sequence_number & 0x0FU);
    memcpy(&frame[1], &s_tx_context.payload[s_tx_context.bytes_sent], chunk);

    status = CanTp_SendCanFrame(frame);
    if (status == CANTP_BUSY) {
        return;
    }
    if (status != CANTP_OK) {
        printf("[ISOTP TX] CF transmit error, abort\r\n");
        CanTp_ResetTxState();
        return;
    }

    s_tx_context.bytes_sent = (uint16_t)(s_tx_context.bytes_sent + chunk);
    s_tx_context.next_sequence_number =
        (uint8_t)((s_tx_context.next_sequence_number + 1U) & 0x0FU);
    s_tx_context.block_counter++;
    s_tx_context.last_tx_tick_ms = now_ms;

    if (s_tx_context.bytes_sent >= s_tx_context.total_length) {
        printf("[ISOTP TX] Multi-frame done, total %u bytes\r\n",
               s_tx_context.total_length);
        CanTp_ResetTxState();
        return;
    }

    if ((s_tx_context.block_size > 0U) &&
        (s_tx_context.block_counter >= s_tx_context.block_size)) {
        s_tx_context.block_counter = 0U;
        s_tx_context.wait_frame_count = 0U;
        s_tx_context.fc_wait_start_tick_ms = now_ms;
        s_tx_context.state = CANTP_TX_WAIT_FC;
    }
}

void CanTp_Init(void)
{
    CanTp_ResetRxState();
    CanTp_ResetTxState();
    s_rx_fc_pending = false;
    s_rx_callback = NULL;
    printf("[ISOTP] Init OK, TX_ID=0x%03X, RX_ID=0x%03X\r\n",
           CANTP_TX_CAN_ID, CANTP_RX_CAN_ID);
}
