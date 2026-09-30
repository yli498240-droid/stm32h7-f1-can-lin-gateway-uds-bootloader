#ifndef NVM_H
#define NVM_H

#include <stdbool.h>
#include <stdint.h>

/* M14 Hardware Gate only. Formal releases must keep this disabled. */
#define M14_NVM_TEST_ENABLE  0U

#define NVM_BLOCK_H7_USER_CONFIG  1U

typedef struct
{
    uint8_t window_switch_enabled;
    uint8_t reserved[7];
} NvM_H7UserConfigType;

typedef enum
{
    NVM_REQ_OK = 0,
    NVM_REQ_PENDING,
    NVM_REQ_NOT_OK,
    NVM_REQ_INTEGRITY_FAILED,
    NVM_REQ_DEFAULTED,
    NVM_REQ_BUSY
} NvM_RequestResultType;

typedef enum
{
    NVM_STATE_UNINIT = 0,
    NVM_STATE_READY,
    NVM_STATE_DEFAULTED,
    NVM_STATE_DIRTY_WAIT,
    NVM_STATE_ERASE_SPARE,
    NVM_STATE_PROGRAM_BODY,
    NVM_STATE_VERIFY_BODY,
    NVM_STATE_PROGRAM_COMMIT,
    NVM_STATE_VERIFY_RECORD,
    NVM_STATE_ERROR
} NvM_StateType;

typedef enum
{
    NVM_ERROR_NONE = 0,
    NVM_ERROR_ARGUMENT,
    NVM_ERROR_BANK_SWAP,
    NVM_ERROR_NO_VALID_RECORD,
    NVM_ERROR_FLASH,
    NVM_ERROR_VERIFY,
    NVM_ERROR_LAYOUT
} NvM_ErrorType;

typedef struct
{
    NvM_StateType state;
    NvM_RequestResultType block_status;
    NvM_ErrorType last_error;
    uint8_t window_switch_enabled;
    uint8_t active_sector;
    uint8_t dirty;
    uint8_t busy;
    uint32_t sequence;
    uint32_t latest_record_address;
    uint32_t next_record_address;
    uint32_t last_flash_hal_error;
} NvM_StatusType;

void NvM_Init(void);
NvM_RequestResultType NvM_ReadBlock(uint16_t block_id, void *destination,
                                    uint16_t length);
NvM_RequestResultType NvM_WriteBlock(uint16_t block_id, const void *source,
                                     uint16_t length);
void NvM_MainFunction(uint32_t now_ms);
NvM_RequestResultType NvM_GetErrorStatus(uint16_t block_id);
bool NvM_IsBusy(void);
NvM_RequestResultType NvM_RestoreBlockDefaults(uint16_t block_id);
void NvM_GetStatus(NvM_StatusType *status);
const char *NvM_StateName(NvM_StateType state);

#if M14_NVM_TEST_ENABLE
bool NvM_TestFormat(void);
bool NvM_TestWriteBodyOnly(void);
bool NvM_TestWriteBadCrcRecord(void);
#endif

#endif /* NVM_H */
