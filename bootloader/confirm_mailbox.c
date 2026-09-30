#include "confirm_mailbox.h"
#include "image_layout.h"
#include "image_validator.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <string.h>
typedef char MailboxSize[(sizeof(ConfirmMailbox_Type)==16U)?1:-1];
typedef char MailboxMagicOffset[(offsetof(ConfirmMailbox_Type,magic)==0U)?1:-1];
typedef char MailboxCrcOffset[(offsetof(ConfirmMailbox_Type,crc32)==12U)?1:-1];
void ConfirmMailbox_Clear(void){*(volatile uint32_t*)CONFIRM_MAILBOX_ADDRESS=0U;__DSB();}
bool ConfirmMailbox_ReadValid(ConfirmMailbox_Type*out){ConfirmMailbox_Type s;memcpy(&s,(const void*)CONFIRM_MAILBOX_ADDRESS,16U);if(out)*out=s;return s.magic==CONFIRM_MAILBOX_MAGIC&&s.version==CONFIRM_MAILBOX_VERSION&&(s.command==CONFIRM_MAILBOX_COMMAND_PENDING||s.command==CONFIRM_MAILBOX_COMMAND_CONFIRM)&&ImageLayout_IsSlotId(s.slot_id)&&s.reserved==0U&&s.crc32==ImageValidator_CalculateCrc32((const uint8_t*)&s,12U);}
void ConfirmMailbox_Write(uint8_t cmd,uint8_t slot,uint32_t seq){ConfirmMailbox_Type s;volatile uint32_t*m=(volatile uint32_t*)CONFIRM_MAILBOX_ADDRESS;memset(&s,0,sizeof(s));s.magic=CONFIRM_MAILBOX_MAGIC;s.version=CONFIRM_MAILBOX_VERSION;s.command=cmd;s.slot_id=slot;s.metadata_sequence=seq;s.crc32=ImageValidator_CalculateCrc32((const uint8_t*)&s,12U);m[0]=0U;__DSB();m[1]=((const uint32_t*)&s)[1];m[2]=((const uint32_t*)&s)[2];m[3]=((const uint32_t*)&s)[3];__DSB();m[0]=CONFIRM_MAILBOX_MAGIC;__DSB();__ISB();}
