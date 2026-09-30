#include "app_boot_contract.h"
#include "app_image_descriptor.h"
#include "m12_test_config.h"
#include "motor_control.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#define MAILBOX_ADDRESS 0x2000BFE0U
#define MAILBOX_MAGIC 0x4332314DU
#define MAILBOX_VERSION 0x01U
#define MAILBOX_PENDING 0x01U
#define MAILBOX_CONFIRM 0x02U
typedef struct {uint32_t magic;uint8_t version,command,slot_id,reserved;uint32_t metadata_sequence,crc32;} Mailbox;
typedef char MailboxSize[(sizeof(Mailbox)==16U)?1:-1];
typedef char MailboxVersionOffset[(offsetof(Mailbox,version)==4U)?1:-1];
typedef char MailboxCommandOffset[(offsetof(Mailbox,command)==5U)?1:-1];
typedef char MailboxSlotOffset[(offsetof(Mailbox,slot_id)==6U)?1:-1];
typedef char MailboxReservedOffset[(offsetof(Mailbox,reserved)==7U)?1:-1];
typedef char MailboxSequenceOffset[(offsetof(Mailbox,metadata_sequence)==8U)?1:-1];
typedef char MailboxCrcOffset[(offsetof(Mailbox,crc32)==12U)?1:-1];
static bool s_armed;static uint32_t s_arm_tick,s_sequence;
static uint32_t Crc(const uint8_t*d,uint32_t n){uint32_t c=0xFFFFFFFFU,i;for(i=0;i<n;i++){uint8_t b;c^=d[i];for(b=0;b<8U;b++)c=(c&1U)?((c>>1U)^0xEDB88320U):(c>>1U);}return ~c;}
static bool Read(Mailbox*out){Mailbox m;memcpy(&m,(const void*)MAILBOX_ADDRESS,16U);if(out)*out=m;return m.magic==MAILBOX_MAGIC&&m.version==MAILBOX_VERSION&&m.command==MAILBOX_PENDING&&m.slot_id==(uint8_t)APP_SLOT_ID&&m.reserved==0U&&m.crc32==Crc((const uint8_t*)&m,12U);}
static void WriteConfirm(uint32_t seq){Mailbox m;volatile uint32_t*p=(volatile uint32_t*)MAILBOX_ADDRESS;memset(&m,0,sizeof(m));m.magic=MAILBOX_MAGIC;m.version=MAILBOX_VERSION;m.command=MAILBOX_CONFIRM;m.slot_id=(uint8_t)APP_SLOT_ID;m.metadata_sequence=seq;m.crc32=Crc((const uint8_t*)&m,12U);p[0]=0U;__DSB();p[1]=((const uint32_t*)&m)[1];p[2]=((const uint32_t*)&m)[2];p[3]=((const uint32_t*)&m)[3];__DSB();p[0]=MAILBOX_MAGIC;__DSB();__ISB();}
void AppBootContract_ArmIfPending(uint32_t now)
{
    Mailbox m;
    s_armed=false;
    if(Read(&m)){
        s_sequence=m.metadata_sequence;
        s_arm_tick=now;
        s_armed=true;
        printf("[APP] Pending Slot %c armed, sequence=%lu\r\n",
               APP_SLOT_ID==0U?'A':'B',(unsigned long)s_sequence);
        if(M12_TEST_SUPPRESS_PENDING_CONFIRM != 0U){
            printf("[APP][M12-TEST] Pending confirm suppressed; reset at cooperative deadline\r\n");
        }
    }
}
bool AppBootContract_IsArmed(void){return s_armed;}
void AppBootContract_MainFunction(uint32_t now,bool healthy)
{
    Mailbox m;
    uint32_t elapsed;
    if(!s_armed)return;
    elapsed=now-s_arm_tick;
    if((M12_TEST_SUPPRESS_PENDING_CONFIRM == 0U) && healthy &&
       (elapsed>=APP_CONFIRM_HEALTHY_DELAY_MS)){
        if(!Read(&m)||m.metadata_sequence!=s_sequence){
            s_armed=false;
            printf("[APP] Pending mailbox changed; confirm cancelled\r\n");
            return;
        }
        WriteConfirm(s_sequence);
        MotorControl_Brake();
        NVIC_SystemReset();
    }
    if(((!healthy)||(M12_TEST_SUPPRESS_PENDING_CONFIRM != 0U)) &&
       (elapsed>=APP_CONFIRM_COOPERATIVE_DEADLINE_MS)){
        s_armed=false;
        MotorControl_Brake();
        NVIC_SystemReset();
    }
}
