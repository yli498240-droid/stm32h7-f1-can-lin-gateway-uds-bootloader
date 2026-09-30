#include "nvm.h"

#include "nvm_flash_h7.h"
#include "stm32h7xx_hal.h"
#include <stddef.h>
#include <string.h>

#define NVM_RECORD_SIZE              64U
#define NVM_BODY_SIZE                32U
#define NVM_RECORDS_PER_SECTOR       (NVM_FLASH_SECTOR_SIZE / NVM_RECORD_SIZE)
#define NVM_MAGIC                    0x314D564EUL /* "NVM1", little endian */
#define NVM_COMMIT_MAGIC             0x54494D43UL /* "CMIT", little endian */
#define NVM_FORMAT_VERSION           1U
#define NVM_PAYLOAD_LENGTH           8U
#define NVM_DIRTY_SETTLE_MS          2000UL
#define NVM_MIN_COMMIT_INTERVAL_MS  10000UL
#define NVM_INVALID_ADDRESS          0xFFFFFFFFUL

#define NVM_OFF_MAGIC                 0U
#define NVM_OFF_VERSION               4U
#define NVM_OFF_BLOCK_ID              6U
#define NVM_OFF_SEQUENCE              8U
#define NVM_OFF_PAYLOAD_LENGTH       12U
#define NVM_OFF_FLAGS                14U
#define NVM_OFF_PAYLOAD              16U
#define NVM_OFF_CRC                  24U
#define NVM_OFF_SEQUENCE_INVERSE     28U
#define NVM_OFF_COMMIT_MAGIC         32U
#define NVM_OFF_COMMIT_SEQUENCE      36U
#define NVM_OFF_CRC_MIRROR           40U
#define NVM_OFF_COMMIT_BLOCK_VERSION 44U

typedef struct
{
    uint8_t found;
    uint8_t sector;
    uint32_t sequence;
    uint32_t address;
    NvM_H7UserConfigType config;
} NvM_RecordCandidateType;

static NvM_StateType s_state;
static NvM_RequestResultType s_block_status;
static NvM_ErrorType s_last_error;
static NvM_H7UserConfigType s_ram_config;
static NvM_H7UserConfigType s_committed_config;
static NvM_H7UserConfigType s_write_config;
static uint8_t s_have_committed_value;
static uint8_t s_active_sector;
static uint8_t s_dirty;
static uint32_t s_sequence;
static uint32_t s_latest_address;
static uint32_t s_next_address;
static uint32_t s_dirty_tick;
static uint32_t s_last_commit_tick;
static uint32_t s_target_address;
static uint8_t s_target_sector;
static uint8_t s_record[NVM_RECORD_SIZE] __ALIGNED(NVM_FLASH_WORD_SIZE);

static uint16_t NvM_GetU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t NvM_GetU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void NvM_PutU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void NvM_PutU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static uint32_t NvM_Crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;

    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 1U) != 0U) ?
                      ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t NvM_SectorStart(uint8_t sector)
{
    return (sector == 0U) ? NVM_FLASH_SECTOR_A_START :
                            NVM_FLASH_SECTOR_B_START;
}

static uint8_t NvM_ConfigValid(const NvM_H7UserConfigType *config)
{
    uint8_t index;

    if ((config == NULL) || (config->window_switch_enabled > 1U)) return 0U;
    for (index = 0U; index < sizeof(config->reserved); index++) {
        if (config->reserved[index] != 0U) return 0U;
    }
    return 1U;
}

static uint8_t NvM_SequenceNewer(uint32_t candidate, uint32_t current)
{
    return ((int32_t)(candidate - current) > 0) ? 1U : 0U;
}

static uint8_t NvM_RecordValid(const uint8_t record[NVM_RECORD_SIZE],
                               NvM_H7UserConfigType *config,
                               uint32_t *sequence)
{
    uint32_t record_sequence;
    uint32_t crc;

    if ((NvM_GetU32(&record[NVM_OFF_MAGIC]) != NVM_MAGIC) ||
        (NvM_GetU16(&record[NVM_OFF_VERSION]) != NVM_FORMAT_VERSION) ||
        (NvM_GetU16(&record[NVM_OFF_BLOCK_ID]) !=
         NVM_BLOCK_H7_USER_CONFIG) ||
        (NvM_GetU16(&record[NVM_OFF_PAYLOAD_LENGTH]) !=
         NVM_PAYLOAD_LENGTH) ||
        (NvM_GetU16(&record[NVM_OFF_FLAGS]) != 0U)) return 0U;

    record_sequence = NvM_GetU32(&record[NVM_OFF_SEQUENCE]);
    if (NvM_GetU32(&record[NVM_OFF_SEQUENCE_INVERSE]) != ~record_sequence) {
        return 0U;
    }
    memcpy(config, &record[NVM_OFF_PAYLOAD], sizeof(*config));
    if (NvM_ConfigValid(config) == 0U) return 0U;

    crc = NvM_Crc32(record, NVM_OFF_CRC);
    if ((NvM_GetU32(&record[NVM_OFF_CRC]) != crc) ||
        (NvM_GetU32(&record[NVM_OFF_COMMIT_MAGIC]) != NVM_COMMIT_MAGIC) ||
        (NvM_GetU32(&record[NVM_OFF_COMMIT_SEQUENCE]) != record_sequence) ||
        (NvM_GetU32(&record[NVM_OFF_CRC_MIRROR]) != crc) ||
        (NvM_GetU32(&record[NVM_OFF_COMMIT_BLOCK_VERSION]) !=
         (((uint32_t)NVM_FORMAT_VERSION << 16) |
          NVM_BLOCK_H7_USER_CONFIG))) return 0U;

    *sequence = record_sequence;
    return 1U;
}

static void NvM_BuildRecord(uint32_t sequence,
                            const NvM_H7UserConfigType *config)
{
    uint32_t crc;

    memset(s_record, 0xFF, sizeof(s_record));
    NvM_PutU32(&s_record[NVM_OFF_MAGIC], NVM_MAGIC);
    NvM_PutU16(&s_record[NVM_OFF_VERSION], NVM_FORMAT_VERSION);
    NvM_PutU16(&s_record[NVM_OFF_BLOCK_ID], NVM_BLOCK_H7_USER_CONFIG);
    NvM_PutU32(&s_record[NVM_OFF_SEQUENCE], sequence);
    NvM_PutU16(&s_record[NVM_OFF_PAYLOAD_LENGTH], NVM_PAYLOAD_LENGTH);
    NvM_PutU16(&s_record[NVM_OFF_FLAGS], 0U);
    memcpy(&s_record[NVM_OFF_PAYLOAD], config, sizeof(*config));
    NvM_PutU32(&s_record[NVM_OFF_SEQUENCE_INVERSE], ~sequence);
    crc = NvM_Crc32(s_record, NVM_OFF_CRC);
    NvM_PutU32(&s_record[NVM_OFF_CRC], crc);
    NvM_PutU32(&s_record[NVM_OFF_COMMIT_MAGIC], NVM_COMMIT_MAGIC);
    NvM_PutU32(&s_record[NVM_OFF_COMMIT_SEQUENCE], sequence);
    NvM_PutU32(&s_record[NVM_OFF_CRC_MIRROR], crc);
    NvM_PutU32(&s_record[NVM_OFF_COMMIT_BLOCK_VERSION],
               ((uint32_t)NVM_FORMAT_VERSION << 16) |
               NVM_BLOCK_H7_USER_CONFIG);
}

static uint32_t NvM_FindFirstBlank(uint8_t sector)
{
    uint32_t slot;
    uint32_t address = NvM_SectorStart(sector);

    for (slot = 0U; slot < NVM_RECORDS_PER_SECTOR; slot++) {
        uint32_t current = address + (slot * NVM_RECORD_SIZE);
        if (NvMFlash_IsErased(current, NVM_RECORD_SIZE)) return current;
    }
    return NVM_INVALID_ADDRESS;
}

static void NvM_ScanSector(uint8_t sector, NvM_RecordCandidateType *best)
{
    uint32_t slot;
    uint32_t start = NvM_SectorStart(sector);
    uint8_t record[NVM_RECORD_SIZE] __ALIGNED(NVM_FLASH_WORD_SIZE);

    for (slot = 0U; slot < NVM_RECORDS_PER_SECTOR; slot++) {
        NvM_H7UserConfigType config;
        uint32_t sequence;
        uint32_t address = start + (slot * NVM_RECORD_SIZE);

        if (NvMFlash_IsErased(address, NVM_RECORD_SIZE)) continue;
        NvMFlash_Read(address, record, sizeof(record));
        if ((NvM_RecordValid(record, &config, &sequence) != 0U) &&
            ((best->found == 0U) ||
             (NvM_SequenceNewer(sequence, best->sequence) != 0U))) {
            best->found = 1U;
            best->sector = sector;
            best->sequence = sequence;
            best->address = address;
            best->config = config;
        }
    }
}

static void NvM_SetFlashError(NvMFlash_ResultType result)
{
    if (result == NVM_FLASH_ERROR_BANK_SWAP) s_last_error = NVM_ERROR_BANK_SWAP;
    else if (result == NVM_FLASH_ERROR_VERIFY) s_last_error = NVM_ERROR_VERIFY;
    else s_last_error = NVM_ERROR_FLASH;
    s_block_status = NVM_REQ_NOT_OK;
    s_state = NVM_STATE_ERROR;
}

static void NvM_PrepareWrite(void)
{
    s_write_config = s_ram_config;
    s_target_sector = s_active_sector;
    s_target_address = NvM_FindFirstBlank(s_active_sector);
    NvM_BuildRecord(s_sequence + 1U, &s_write_config);

    if (s_target_address == NVM_INVALID_ADDRESS) {
        s_target_sector = (s_active_sector == 0U) ? 1U : 0U;
        s_target_address = NvM_SectorStart(s_target_sector);
        s_state = NVM_STATE_ERASE_SPARE;
    } else {
        s_state = NVM_STATE_PROGRAM_BODY;
    }
}

void NvM_Init(void)
{
    NvM_RecordCandidateType best = {0};

    memset(&s_ram_config, 0, sizeof(s_ram_config));
    s_ram_config.window_switch_enabled = 1U;
    s_committed_config = s_ram_config;
    s_write_config = s_ram_config;
    s_have_committed_value = 0U;
    s_active_sector = 0U;
    s_dirty = 0U;
    s_sequence = 0U;
    s_latest_address = NVM_INVALID_ADDRESS;
    s_next_address = NVM_FLASH_SECTOR_A_START;
    s_dirty_tick = 0U;
    s_last_commit_tick = 0U;
    s_last_error = NVM_ERROR_NONE;
    s_block_status = NVM_REQ_DEFAULTED;
    s_state = NVM_STATE_DEFAULTED;

    if (!NvMFlash_IsBankSwapDisabled()) {
        s_last_error = NVM_ERROR_BANK_SWAP;
        s_block_status = NVM_REQ_NOT_OK;
        s_state = NVM_STATE_ERROR;
        return;
    }

    NvM_ScanSector(0U, &best);
    NvM_ScanSector(1U, &best);
    if (best.found != 0U) {
        s_ram_config = best.config;
        s_committed_config = best.config;
        s_have_committed_value = 1U;
        s_active_sector = best.sector;
        s_sequence = best.sequence;
        s_latest_address = best.address;
        s_next_address = NvM_FindFirstBlank(best.sector);
        s_block_status = NVM_REQ_OK;
        s_state = NVM_STATE_READY;
    } else {
        s_last_error = NVM_ERROR_NO_VALID_RECORD;
        s_next_address = NvM_FindFirstBlank(0U);
    }
}

NvM_RequestResultType NvM_ReadBlock(uint16_t block_id, void *destination,
                                    uint16_t length)
{
    if ((block_id != NVM_BLOCK_H7_USER_CONFIG) || (destination == NULL) ||
        (length != sizeof(s_ram_config))) return NVM_REQ_NOT_OK;
    memcpy(destination, &s_ram_config, sizeof(s_ram_config));
    return s_block_status;
}

NvM_RequestResultType NvM_WriteBlock(uint16_t block_id, const void *source,
                                     uint16_t length)
{
    const NvM_H7UserConfigType *config = source;

    if ((block_id != NVM_BLOCK_H7_USER_CONFIG) || (source == NULL) ||
        (length != sizeof(s_ram_config)) || (NvM_ConfigValid(config) == 0U)) {
        s_last_error = NVM_ERROR_ARGUMENT;
        return NVM_REQ_NOT_OK;
    }
    if ((s_state == NVM_STATE_UNINIT) ||
        ((s_state == NVM_STATE_ERROR) &&
         (s_last_error != NVM_ERROR_FLASH) &&
         (s_last_error != NVM_ERROR_VERIFY))) {
        return NVM_REQ_NOT_OK;
    }
    if (NvM_IsBusy() && (s_state != NVM_STATE_DIRTY_WAIT)) {
        /* The live value wins immediately; the in-flight snapshot commits first,
           then the latest value is coalesced into a following record. */
        s_ram_config = *config;
        s_dirty = 1U;
        s_dirty_tick = HAL_GetTick();
        s_block_status = NVM_REQ_PENDING;
        return NVM_REQ_PENDING;
    }
    if (s_state == NVM_STATE_ERROR) {
        s_ram_config = *config;
        s_dirty = 1U;
        s_dirty_tick = HAL_GetTick();
        s_state = NVM_STATE_DIRTY_WAIT;
        s_block_status = NVM_REQ_PENDING;
        return NVM_REQ_PENDING;
    }
    if (memcmp(&s_ram_config, config, sizeof(*config)) == 0) {
        return (s_dirty != 0U) ? NVM_REQ_PENDING : s_block_status;
    }

    s_ram_config = *config;
    if ((s_have_committed_value != 0U) &&
        (memcmp(&s_ram_config, &s_committed_config,
                sizeof(s_ram_config)) == 0)) {
        s_dirty = 0U;
        s_state = NVM_STATE_READY;
        s_block_status = NVM_REQ_OK;
        return NVM_REQ_OK;
    }

    s_dirty = 1U;
    s_dirty_tick = HAL_GetTick();
    s_state = NVM_STATE_DIRTY_WAIT;
    s_block_status = NVM_REQ_PENDING;
    return NVM_REQ_PENDING;
}

void NvM_MainFunction(uint32_t now_ms)
{
    NvMFlash_ResultType flash_result;

    switch (s_state) {
        case NVM_STATE_DIRTY_WAIT:
            if (((uint32_t)(now_ms - s_dirty_tick) >= NVM_DIRTY_SETTLE_MS) &&
                ((uint32_t)(now_ms - s_last_commit_tick) >=
                 NVM_MIN_COMMIT_INTERVAL_MS)) {
                NvM_PrepareWrite();
            }
            break;

        case NVM_STATE_ERASE_SPARE:
            flash_result = NvMFlash_EraseSector(
                NvM_SectorStart(s_target_sector));
            if (flash_result == NVM_FLASH_OK) {
                s_state = NVM_STATE_PROGRAM_BODY;
            } else {
                NvM_SetFlashError(flash_result);
            }
            break;

        case NVM_STATE_PROGRAM_BODY:
            flash_result = NvMFlash_ProgramWord(s_target_address, s_record);
            if (flash_result == NVM_FLASH_OK) s_state = NVM_STATE_VERIFY_BODY;
            else NvM_SetFlashError(flash_result);
            break;

        case NVM_STATE_VERIFY_BODY:
            if (memcmp((const void *)s_target_address, s_record,
                       NVM_BODY_SIZE) == 0) {
                s_state = NVM_STATE_PROGRAM_COMMIT;
            } else {
                NvM_SetFlashError(NVM_FLASH_ERROR_VERIFY);
            }
            break;

        case NVM_STATE_PROGRAM_COMMIT:
            flash_result = NvMFlash_ProgramWord(s_target_address + NVM_BODY_SIZE,
                                                &s_record[NVM_BODY_SIZE]);
            if (flash_result == NVM_FLASH_OK) s_state = NVM_STATE_VERIFY_RECORD;
            else NvM_SetFlashError(flash_result);
            break;

        case NVM_STATE_VERIFY_RECORD:
        {
            NvM_H7UserConfigType verified;
            uint32_t verified_sequence;
            uint8_t readback[NVM_RECORD_SIZE] __ALIGNED(NVM_FLASH_WORD_SIZE);

            NvMFlash_Read(s_target_address, readback, sizeof(readback));
            if ((NvM_RecordValid(readback, &verified,
                                 &verified_sequence) == 0U) ||
                (memcmp(&verified, &s_write_config, sizeof(verified)) != 0)) {
                NvM_SetFlashError(NVM_FLASH_ERROR_VERIFY);
                break;
            }
            s_sequence = verified_sequence;
            s_latest_address = s_target_address;
            s_active_sector = s_target_sector;
            s_next_address = NvM_FindFirstBlank(s_active_sector);
            s_committed_config = verified;
            s_have_committed_value = 1U;
            s_last_commit_tick = now_ms;
            s_last_error = NVM_ERROR_NONE;
            if (memcmp(&s_ram_config, &s_committed_config,
                       sizeof(s_ram_config)) != 0) {
                s_dirty = 1U;
                s_block_status = NVM_REQ_PENDING;
                s_state = NVM_STATE_DIRTY_WAIT;
            } else {
                s_dirty = 0U;
                s_block_status = NVM_REQ_OK;
                s_state = NVM_STATE_READY;
            }
            break;
        }

        default:
            break;
    }
}

NvM_RequestResultType NvM_GetErrorStatus(uint16_t block_id)
{
    return (block_id == NVM_BLOCK_H7_USER_CONFIG) ? s_block_status :
                                                    NVM_REQ_NOT_OK;
}

bool NvM_IsBusy(void)
{
    return (s_state == NVM_STATE_DIRTY_WAIT) ||
           (s_state == NVM_STATE_ERASE_SPARE) ||
           (s_state == NVM_STATE_PROGRAM_BODY) ||
           (s_state == NVM_STATE_VERIFY_BODY) ||
           (s_state == NVM_STATE_PROGRAM_COMMIT) ||
           (s_state == NVM_STATE_VERIFY_RECORD);
}

NvM_RequestResultType NvM_RestoreBlockDefaults(uint16_t block_id)
{
    NvM_H7UserConfigType defaults = {1U, {0U}};
    if (block_id != NVM_BLOCK_H7_USER_CONFIG) return NVM_REQ_NOT_OK;
    return NvM_WriteBlock(block_id, &defaults, sizeof(defaults));
}

void NvM_GetStatus(NvM_StatusType *status)
{
    if (status == NULL) return;
    status->state = s_state;
    status->block_status = s_block_status;
    status->last_error = s_last_error;
    status->window_switch_enabled = s_ram_config.window_switch_enabled;
    status->active_sector = s_active_sector;
    status->dirty = s_dirty;
    status->busy = NvM_IsBusy() ? 1U : 0U;
    status->sequence = s_sequence;
    status->latest_record_address = s_latest_address;
    status->next_record_address = s_next_address;
    status->last_flash_hal_error = NvMFlash_GetLastHalError();
}

const char *NvM_StateName(NvM_StateType state)
{
    switch (state) {
        case NVM_STATE_UNINIT: return "UNINIT";
        case NVM_STATE_READY: return "READY";
        case NVM_STATE_DEFAULTED: return "DEFAULTED";
        case NVM_STATE_DIRTY_WAIT: return "DIRTY_WAIT";
        case NVM_STATE_ERASE_SPARE: return "ERASE_SPARE";
        case NVM_STATE_PROGRAM_BODY: return "PROGRAM_BODY";
        case NVM_STATE_VERIFY_BODY: return "VERIFY_BODY";
        case NVM_STATE_PROGRAM_COMMIT: return "PROGRAM_COMMIT";
        case NVM_STATE_VERIFY_RECORD: return "VERIFY_RECORD";
        case NVM_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

#if M14_NVM_TEST_ENABLE
bool NvM_TestFormat(void)
{
    if (NvM_IsBusy()) return false;
    if (NvMFlash_EraseSector(NVM_FLASH_SECTOR_A_START) != NVM_FLASH_OK) return false;
    if (NvMFlash_EraseSector(NVM_FLASH_SECTOR_B_START) != NVM_FLASH_OK) return false;
    NvM_Init();
    return true;
}

bool NvM_TestWriteBodyOnly(void)
{
    uint32_t address;

    if (NvM_IsBusy() || (s_have_committed_value == 0U)) return false;
    address = NvM_FindFirstBlank(s_active_sector);
    if (address == NVM_INVALID_ADDRESS) return false;
    NvM_BuildRecord(s_sequence + 1U, &s_ram_config);
    return NvMFlash_ProgramWord(address, s_record) == NVM_FLASH_OK;
}

bool NvM_TestWriteBadCrcRecord(void)
{
    uint32_t address;

    if (NvM_IsBusy() || (s_have_committed_value == 0U)) return false;
    address = NvM_FindFirstBlank(s_active_sector);
    if (address == NVM_INVALID_ADDRESS) return false;
    NvM_BuildRecord(s_sequence + 1U, &s_ram_config);
    s_record[NVM_OFF_CRC] ^= 0x01U;
    if (NvMFlash_ProgramWord(address, s_record) != NVM_FLASH_OK) return false;
    return NvMFlash_ProgramWord(address + NVM_BODY_SIZE,
                               &s_record[NVM_BODY_SIZE]) == NVM_FLASH_OK;
}
#endif
