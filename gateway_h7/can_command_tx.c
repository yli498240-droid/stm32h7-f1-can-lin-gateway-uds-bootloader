#include "can_command_tx.h"
#include "can_if.h"

#include <stdio.h>

CanCommandTx_Status_t CanCommandTx_Send(uint8_t command)
{
    CanIf_PduType pdu = {0};
    CanIf_StatusType status;
    static const char *command_names[] = {"STOP", "OPEN", "CLOSE", "RESET"};
    const char *name = (command <= 3U) ? command_names[command] : "???";

    pdu.id = 0x100U;
    pdu.dlc = 1U;
    pdu.data[0] = command;

    status = CanIf_Transmit(&pdu);
    if (status != CANIF_OK) {
        printf("\r\n[CMD TX] FAILED -> %s (0x%02X), CANIF=%lu\r\n",
               name, command, (unsigned long)status);
        return CAN_COMMAND_TX_ERROR;
    }

    return CAN_COMMAND_TX_OK;
}
