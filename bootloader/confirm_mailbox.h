#ifndef CONFIRM_MAILBOX_H
#define CONFIRM_MAILBOX_H
#include <stdbool.h>
#include <stdint.h>
#define CONFIRM_MAILBOX_ADDRESS 0x2000BFE0U
#define CONFIRM_MAILBOX_MAGIC 0x4332314DU
#define CONFIRM_MAILBOX_VERSION 0x01U
#define CONFIRM_MAILBOX_COMMAND_PENDING 0x01U
#define CONFIRM_MAILBOX_COMMAND_CONFIRM 0x02U
typedef struct { uint32_t magic; uint8_t version,command,slot_id,reserved; uint32_t metadata_sequence,crc32; } ConfirmMailbox_Type;
bool ConfirmMailbox_ReadValid(ConfirmMailbox_Type *snapshot);
void ConfirmMailbox_Clear(void);
void ConfirmMailbox_Write(uint8_t command,uint8_t slot_id,uint32_t sequence);
#endif
