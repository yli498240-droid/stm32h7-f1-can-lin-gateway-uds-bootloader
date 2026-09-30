#include "boot_manager.h"
#include "main.h"
#include "app_image_descriptor.h"
#include "confirm_mailbox.h"
#include "dem_event_adapter.h"
#include <stdio.h>
#include <string.h>
#define BOOT_MAGIC_ADDRESS 0x2000BFFCU
#define BOOT_MAGIC_VALUE 0xDEADBEEFU
static bool s_jump_authorized;
static uint32_t s_authorized_base;
static ImageValidator_VectorType s_authorized_vector;

__attribute__((noreturn,noinline)) static void Transfer(uint32_t msp,uint32_t reset){__asm volatile("msr msp, %0\n""bx %1\n"::"r"(msp),"r"(reset):"memory");__builtin_unreachable();}
static void MotorSafe(void){HAL_GPIO_WritePin(BOOT_SAFE_PWM_GPIO_Port,BOOT_SAFE_PWM_Pin|BOOT_SAFE_INB_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(BOOT_SAFE_INA_GPIO_Port,BOOT_SAFE_INA_Pin,GPIO_PIN_RESET);}
void BootManager_Init(void){MotorSafe();}

BootManager_ImageCheckType BootManager_ValidateSlot(uint8_t slot,uint32_t size,uint32_t crc,const uint8_t version[4],ImageValidator_VectorType*out)
{
 AppImageDescriptorType d;ImageValidator_VectorType v;uint32_t actual,start;
 uint32_t base=ImageLayout_GetSlotBase(slot);
 if(!ImageValidator_IsImageRangeValidForSlot(slot,base,size))return BOOT_IMAGE_VECTOR_INVALID;
 v.initial_msp=*(volatile const uint32_t*)base;v.reset_handler=*(volatile const uint32_t*)(base+4U);
 if(!ImageValidator_ValidateVectorForImage(v.initial_msp,v.reset_handler,base,size))return BOOT_IMAGE_VECTOR_INVALID;
 if(!AppImageDescriptor_ReadAndValidate(base,size,&d)||!version||memcmp(d.software_version,version,4U)!=0)return BOOT_IMAGE_DESCRIPTOR_INVALID;
 start=HAL_GetTick();actual=ImageValidator_CalculateCrc32((const uint8_t*)base,size);
 printf("[BOOT] Slot %c CRC size=%lu expected=0x%08lX actual=0x%08lX time=%lu ms\r\n",slot==0U?'A':'B',(unsigned long)size,(unsigned long)crc,(unsigned long)actual,(unsigned long)(HAL_GetTick()-start));
 DemEventAdapter_ReportCrcResult(actual==crc,HAL_GetTick());if(actual!=crc)return BOOT_IMAGE_CRC_INVALID;if(out)*out=v;return BOOT_IMAGE_VALID;
}
static ImageMetadata_RollbackReasonType PendingReason(BootManager_ImageCheckType c){return c==BOOT_IMAGE_VECTOR_INVALID?IMAGE_ROLLBACK_PENDING_VECTOR_INVALID:c==BOOT_IMAGE_DESCRIPTOR_INVALID?IMAGE_ROLLBACK_PENDING_DESCRIPTOR_INVALID:IMAGE_ROLLBACK_PENDING_CRC_INVALID;}
static bool Authorize(const ImageMetadata_RecordType*r,uint8_t slot,ImageValidator_VectorType*out)
{ImageValidator_VectorType v;BootManager_ImageCheckType c=BootManager_ValidateSlot(slot,ImageMetadata_GetSlotSize(r,slot),ImageMetadata_GetSlotCrc(r,slot),ImageMetadata_GetSlotVersion(r,slot),&v);if(c!=BOOT_IMAGE_VALID)return false;s_authorized_vector=v;s_authorized_base=ImageLayout_GetSlotBase(slot);s_jump_authorized=true;if(out)*out=v;return true;}

static bool ConsumeConfirm(void)
{
 ConfirmMailbox_Type m;bool valid=ConfirmMailbox_ReadValid(&m);
 if(!valid){if(m.magic!=0U)ConfirmMailbox_Clear();return true;}
 if(m.command!=CONFIRM_MAILBOX_COMMAND_CONFIRM){ConfirmMailbox_Clear();return true;}
 ConfirmMailbox_Clear();
 if(ImageMetadata_ConfirmPending(m.slot_id,m.metadata_sequence)!=IMAGE_METADATA_OK){printf("[BOOT] Confirm commit failed; recovery\r\n");return false;}
 printf("[BOOT] Slot %c confirmed\r\n",m.slot_id==0U?'A':'B');return true;
}

static bool MigrateIfNeeded(ImageMetadata_ScanResultType*s)
{
 if(s->state!=IMAGE_METADATA_SCAN_V1_RECORD)return true;
 if(s->legacy.update_state!=2U||BootManager_ValidateSlot(IMAGE_SLOT_A_ID,s->legacy.image_size,s->legacy.image_crc32,s->legacy.software_version,NULL)!=BOOT_IMAGE_VALID){printf("[BOOT] M11 v1 is not a valid migration source\r\n");return false;}
 if(ImageMetadata_MigrateV1(s)!=IMAGE_METADATA_OK){printf("[BOOT] M11->M12 metadata migration failed\r\n");return false;}
 printf("[BOOT] M11 v1 migrated: confirmed Slot A\r\n");ImageMetadata_Scan(s);return s->state==IMAGE_METADATA_SCAN_V2_RECORD;
}

static BootManager_StartupActionType Fallback(ImageMetadata_RecordType*r,ImageValidator_VectorType*out)
{
 uint8_t c=r->confirmed_slot,other=ImageLayout_GetInactiveSlot(c);
 if(ImageLayout_IsSlotId(c)&&ImageMetadata_GetSlotState(r,c)==IMAGE_SLOT_STATE_CONFIRMED&&Authorize(r,c,out)){ConfirmMailbox_Clear();return BOOT_MANAGER_JUMP_APP;}
 if(ImageLayout_IsSlotId(other)&&ImageMetadata_GetSlotState(r,other)==IMAGE_SLOT_STATE_VALID_ROLLBACK&&Authorize(r,other,out)){
   if(ImageMetadata_PromoteRollback(c,other)!=IMAGE_METADATA_OK){s_jump_authorized=false;return BOOT_MANAGER_RECOVERY;}
   return BOOT_MANAGER_JUMP_APP;
 }
 s_jump_authorized=false;return BOOT_MANAGER_RECOVERY;
}

BootManager_StartupActionType BootManager_EvaluateStartup(ImageValidator_VectorType*out)
{
 ImageMetadata_ScanResultType s;ImageMetadata_RecordType r;ImageMetadata_RecordType attempted;
 volatile uint32_t*magic=(volatile uint32_t*)BOOT_MAGIC_ADDRESS;uint8_t p;BootManager_ImageCheckType check;
 s_jump_authorized=false;s_authorized_base=0U;if(out)memset(out,0,sizeof(*out));
 if(!ConsumeConfirm())return BOOT_MANAGER_RECOVERY;
 ImageMetadata_Scan(&s);if(!MigrateIfNeeded(&s))return BOOT_MANAGER_RECOVERY;
 if(*magic==BOOT_MAGIC_VALUE){*magic=0U;printf("[BOOT] Magic detected, entering OTA mode\r\n");return BOOT_MANAGER_START_OTA;}
 if(s.state==IMAGE_METADATA_ABSENT_LEGACY){printf("[BOOT] Metadata absent; recovery required for safe A/B state\r\n");return BOOT_MANAGER_RECOVERY;}
 if(s.state!=IMAGE_METADATA_SCAN_V2_RECORD){printf("[BOOT] Metadata corrupt, fail-closed recovery\r\n");return BOOT_MANAGER_RECOVERY;}
 r=s.record;
 for(p=0U;p<2U;p++)if(ImageMetadata_GetSlotState(&r,p)==IMAGE_SLOT_STATE_UPDATE_IN_PROGRESS){printf("[BOOT] Slot %c update interrupted\r\n",p?'B':'A');if(ImageMetadata_MarkCandidateInvalid(p,IMAGE_ROLLBACK_UPDATE_INTERRUPTED)!=IMAGE_METADATA_OK)return BOOT_MANAGER_RECOVERY;ImageMetadata_Scan(&s);r=s.record;}
 if(ImageLayout_IsSlotId(r.pending_slot)){
   p=r.pending_slot;
   if(r.boot_attempt_count>=IMAGE_METADATA_MAX_BOOT_ATTEMPTS){printf("[BOOT] Pending attempts exceeded\r\n");if(ImageMetadata_MarkCandidateInvalid(p,IMAGE_ROLLBACK_BOOT_ATTEMPTS_EXCEEDED)!=IMAGE_METADATA_OK)return BOOT_MANAGER_RECOVERY;ImageMetadata_Scan(&s);return Fallback(&s.record,out);}
   if(ImageMetadata_IncrementPendingAttempt(p,&attempted)!=IMAGE_METADATA_OK)return BOOT_MANAGER_RECOVERY;
   check=BootManager_ValidateSlot(p,ImageMetadata_GetSlotSize(&attempted,p),ImageMetadata_GetSlotCrc(&attempted,p),ImageMetadata_GetSlotVersion(&attempted,p),&s_authorized_vector);
   if(check!=BOOT_IMAGE_VALID){printf("[BOOT] Pending Slot %c validation failed\r\n",p?'B':'A');if(ImageMetadata_MarkCandidateInvalid(p,PendingReason(check))!=IMAGE_METADATA_OK)return BOOT_MANAGER_RECOVERY;ImageMetadata_Scan(&s);return Fallback(&s.record,out);}
   s_authorized_base=ImageLayout_GetSlotBase(p);s_jump_authorized=true;if(out)*out=s_authorized_vector;ConfirmMailbox_Write(CONFIRM_MAILBOX_COMMAND_PENDING,p,attempted.sequence);printf("[BOOT] Pending Slot %c attempt %u\r\n",p?'B':'A',attempted.boot_attempt_count);return BOOT_MANAGER_JUMP_APP;
 }
 return Fallback(&r,out);
}

bool BootManager_GetDownloadTarget(uint8_t*slot,uint32_t*base)
{
 ImageMetadata_ScanResultType s;uint8_t target;if(!slot||!base)return false;ImageMetadata_Scan(&s);
 if(s.state==IMAGE_METADATA_SCAN_V2_RECORD){ImageMetadata_RecordType*r=&s.record;if(ImageLayout_IsSlotId(r->confirmed_slot)){target=ImageLayout_GetInactiveSlot(r->confirmed_slot);if(!ImageLayout_IsSlotId(target))return false;}else{uint8_t a=r->slot_a_state,b=r->slot_b_state;if(a==IMAGE_SLOT_STATE_VALID_ROLLBACK&&b!=IMAGE_SLOT_STATE_VALID_ROLLBACK)target=IMAGE_SLOT_B_ID;else if(b==IMAGE_SLOT_STATE_VALID_ROLLBACK&&a!=IMAGE_SLOT_STATE_VALID_ROLLBACK)target=IMAGE_SLOT_A_ID;else if(a==IMAGE_SLOT_STATE_PENDING||a==IMAGE_SLOT_STATE_UPDATE_IN_PROGRESS||b==IMAGE_SLOT_STATE_PENDING||b==IMAGE_SLOT_STATE_UPDATE_IN_PROGRESS)return false;else target=IMAGE_SLOT_A_ID;}}
 else if(s.state==IMAGE_METADATA_ABSENT_LEGACY)target=IMAGE_SLOT_A_ID;else return false;*slot=target;*base=ImageLayout_GetSlotBase(target);return true;
}
void BootManager_PrepareOta(void){MotorSafe();}
bool BootManager_TryJumpToApp(void)
{
    volatile uint32_t k;
    MotorSafe();
    if (!s_jump_authorized) {
        printf("[BOOT] Jump denied\r\n");
        return false;
    }
    printf("[BOOT] Jumping Slot @0x%08lX\r\n",
           (unsigned long)s_authorized_base);
    for (k = 0U; k < 1000000U; k++) {
        /* Preserve the established UART drain delay. */
    }
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    NVIC_DisableIRQ(TIM4_IRQn);
    SCB->VTOR = s_authorized_base;
    Transfer(s_authorized_vector.initial_msp,
             s_authorized_vector.reset_handler);
}
void BootManager_ResetToApp(void){HAL_Delay(50U);NVIC_SystemReset();}
