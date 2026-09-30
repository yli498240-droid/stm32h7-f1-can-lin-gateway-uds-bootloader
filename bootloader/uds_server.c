#include "uds_server.h"
#include "dcm.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "flash_program.h"
#include "app_image_descriptor.h"
#include "image_validator.h"
#include "image_metadata.h"
#include "boot_manager.h"
#include "dem_event_adapter.h"

/* Public showcase placeholders; original lab demo constants intentionally omitted. */
#define PUBLIC_SHOWCASE_FIXED_SEED   0x00000001U
#define PUBLIC_SHOWCASE_KEY_XOR_MASK 0x00000000U

/* ============ 内部状态 ============ */
static uds_server_state_t g_state = UDS_SRV_DEFAULT_SESSION;

/* 响应缓冲区 */
static uint8_t g_resp_buf[256];

/* Security */
static uint32_t g_seed = 0;
static bool     g_seed_requested = false;

/* Download */
#define MAX_BLOCK_SIZE   512

static uint32_t g_download_addr = 0;
static uint32_t g_download_size = 0;
static uint32_t g_download_received = 0;
static uint8_t  g_expected_block_seq = 1;
static bool     g_target_locked = false;
static uint8_t  g_target_slot = IMAGE_SLOT_NONE;
static uint32_t g_target_base = 0U;

/* Boot 端不再用 RAM 暂存，直接写 Flash */

/* Pending Reset */
static volatile bool g_reset_pending = false;
static volatile uint32_t g_reset_tick = 0;

/* ============ 辅助函数 ============ */
static void send_negative_response(uint8_t req_sid, uint8_t nrc)
{
    g_resp_buf[0] = UDS_NEG_RESPONSE;
    g_resp_buf[1] = req_sid;
    g_resp_buf[2] = nrc;
    printf("[UDS-S] -> NEG: SID=0x%02X NRC=0x%02X\r\n", req_sid, nrc);
    (void)Dcm_SendResponse(g_resp_buf, 3U);
}

static void send_positive_response(const uint8_t *data, uint16_t len)
{
    (void)Dcm_SendResponse(data, len);
}

/* ============ 0x10 DiagnosticSessionControl ============ */
static void handle_session_control(const uint8_t *req, uint16_t len)
{
    if (len != 2) {
        send_negative_response(UDS_SID_DIAG_SESSION_CTRL, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    uint8_t session_type = req[1];
    
    if (session_type != UDS_SESSION_DEFAULT &&
        session_type != UDS_SESSION_PROGRAMMING &&
        session_type != UDS_SESSION_EXTENDED) {
        send_negative_response(UDS_SID_DIAG_SESSION_CTRL, NRC_SUB_FUNCTION_NOT_SUPPORTED);
        return;
    }
    
    g_seed_requested = false;
    g_download_received = 0;
    g_target_locked = false;
    g_target_slot = IMAGE_SLOT_NONE;
    g_target_base = 0U;
    
    switch (session_type) {
        case UDS_SESSION_DEFAULT:
            g_state = UDS_SRV_DEFAULT_SESSION;
            printf("[UDS-S] State -> DEFAULT_SESSION\r\n");
            break;
        case UDS_SESSION_PROGRAMMING:
            g_state = UDS_SRV_PROGRAMMING_SESSION;
            printf("[UDS-S] State -> PROGRAMMING_SESSION\r\n");
            break;
        case UDS_SESSION_EXTENDED:
            g_state = UDS_SRV_EXTENDED_SESSION;
            printf("[UDS-S] State -> EXTENDED_SESSION\r\n");
            break;
    }
    
    g_resp_buf[0] = UDS_SID_DIAG_SESSION_CTRL + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = session_type;
    g_resp_buf[2] = 0x00; g_resp_buf[3] = 0x32;
    g_resp_buf[4] = 0x01; g_resp_buf[5] = 0xF4;
    
    send_positive_response(g_resp_buf, 6);
    printf("[UDS-S] -> 50 %02X (Session OK)\r\n", session_type);
}

/* ============ 0x22 ReadDataByIdentifier: Boot-authorized target ============ */
static void handle_read_data_by_id(const uint8_t *req, uint16_t len)
{
    uint16_t did;
    uint16_t page_index;
    uint8_t slot;
    uint32_t base;

    if (len != 3U) {
        send_negative_response(UDS_SID_READ_DATA_BY_ID, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    did = ((uint16_t)req[1] << 8U) | req[2];
    if (did != UDS_DID_OTA_SLOT_TARGET) {
        send_negative_response(UDS_SID_READ_DATA_BY_ID, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    if ((g_state == UDS_SRV_DEFAULT_SESSION) ||
        (g_state == UDS_SRV_EXTENDED_SESSION)) {
        send_negative_response(UDS_SID_READ_DATA_BY_ID, NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);
        return;
    }
    if (g_state != UDS_SRV_SECURITY_UNLOCKED) {
        send_negative_response(UDS_SID_READ_DATA_BY_ID, NRC_SECURITY_ACCESS_DENIED);
        return;
    }
    if (!BootManager_GetDownloadTarget(&slot, &base) ||
        !ImageLayout_IsSlotId(slot) || (base < IMAGE_FLASH_BASE) ||
        (((base - IMAGE_FLASH_BASE) % IMAGE_FLASH_PAGE_SIZE) != 0U)) {
        send_negative_response(UDS_SID_READ_DATA_BY_ID, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    page_index = (uint16_t)((base - IMAGE_FLASH_BASE) / IMAGE_FLASH_PAGE_SIZE);
    g_target_locked = true;
    g_target_slot = slot;
    g_target_base = base;
    g_resp_buf[0] = UDS_SID_READ_DATA_BY_ID + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = 0xF1U;
    g_resp_buf[2] = 0xA0U;
    g_resp_buf[3] = slot;
    g_resp_buf[4] = (uint8_t)(page_index >> 8U);
    g_resp_buf[5] = (uint8_t)page_index;
    send_positive_response(g_resp_buf, 6U);
    printf("[UDS-S] Target locked: Slot %c @0x%08lX\r\n",
           slot == IMAGE_SLOT_A_ID ? 'A' : 'B', (unsigned long)base);
}

/* ============ 0x27 SecurityAccess ============ */
static void handle_security_access(const uint8_t *req, uint16_t len)
{
    if (len < 2) {
        send_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    if (g_state != UDS_SRV_PROGRAMMING_SESSION &&
        g_state != UDS_SRV_SECURITY_UNLOCKED &&
        g_state != UDS_SRV_DOWNLOAD_ACTIVE &&
        g_state != UDS_SRV_TRANSFERRING &&
        g_state != UDS_SRV_TRANSFER_DONE) {
        send_negative_response(UDS_SID_SECURITY_ACCESS, NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);
        return;
    }
    
    uint8_t subfunc = req[1];
    
    if (subfunc == UDS_SEC_REQUEST_SEED) {
        if (len != 2) {
            send_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INCORRECT_MESSAGE_LENGTH);
            return;
        }
        
        if (g_state == UDS_SRV_SECURITY_UNLOCKED ||
            g_state == UDS_SRV_DOWNLOAD_ACTIVE ||
            g_state == UDS_SRV_TRANSFERRING ||
            g_state == UDS_SRV_TRANSFER_DONE) {
            g_seed = 0;
        } else {
            g_seed = PUBLIC_SHOWCASE_FIXED_SEED;
        }
        g_seed_requested = true;
        
        g_resp_buf[0] = UDS_SID_SECURITY_ACCESS + UDS_POS_RESPONSE_OFFSET;
        g_resp_buf[1] = UDS_SEC_REQUEST_SEED;
        g_resp_buf[2] = (g_seed >> 24) & 0xFF;
        g_resp_buf[3] = (g_seed >> 16) & 0xFF;
        g_resp_buf[4] = (g_seed >> 8)  & 0xFF;
        g_resp_buf[5] = g_seed & 0xFF;
        
        send_positive_response(g_resp_buf, 6);
        printf("[UDS-S] -> Seed = 0x%08lX\r\n", (unsigned long)g_seed);
    }
    else if (subfunc == UDS_SEC_SEND_KEY) {
        if (len != 6) {
            send_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INCORRECT_MESSAGE_LENGTH);
            return;
        }
        if (!g_seed_requested) {
            send_negative_response(UDS_SID_SECURITY_ACCESS, NRC_REQUEST_SEQUENCE_ERROR);
            return;
        }
        
        uint32_t key = ((uint32_t)req[2] << 24) |
                       ((uint32_t)req[3] << 16) |
                       ((uint32_t)req[4] << 8)  |
                       ((uint32_t)req[5]);
        
        uint32_t expected = (~g_seed) ^ PUBLIC_SHOWCASE_KEY_XOR_MASK;
        
        if (key != expected) {
            printf("[UDS-S] Key MISMATCH: got 0x%08lX, expected 0x%08lX\r\n",
                   (unsigned long)key, (unsigned long)expected);
            send_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INVALID_KEY);
            g_seed_requested = false;
            return;
        }
        
        g_state = UDS_SRV_SECURITY_UNLOCKED;
        g_seed_requested = false;
        
        g_resp_buf[0] = UDS_SID_SECURITY_ACCESS + UDS_POS_RESPONSE_OFFSET;
        g_resp_buf[1] = UDS_SEC_SEND_KEY;
        send_positive_response(g_resp_buf, 2);
        printf("[UDS-S] -> Key OK, State -> SECURITY_UNLOCKED\r\n");
    }
    else {
        send_negative_response(UDS_SID_SECURITY_ACCESS, NRC_SUB_FUNCTION_NOT_SUPPORTED);
    }
}

/* ============ 0x34 RequestDownload ============ */
static void handle_request_download(const uint8_t *req, uint16_t len)
{
    if (g_state != UDS_SRV_SECURITY_UNLOCKED) {
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_SECURITY_ACCESS_DENIED);
        return;
    }
    
    if (len != 11) {
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    uint8_t alfid = req[2];
    if (alfid != 0x44) {
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    uint32_t addr = ((uint32_t)req[3] << 24) | ((uint32_t)req[4] << 16) |
                    ((uint32_t)req[5] << 8)  | ((uint32_t)req[6]);
    uint32_t size = ((uint32_t)req[7] << 24) | ((uint32_t)req[8] << 16) |
                    ((uint32_t)req[9] << 8)  | ((uint32_t)req[10]);
    
    if (!g_target_locked || (addr != g_target_base) ||
        !ImageValidator_IsImageRangeValidForSlot(g_target_slot, addr, size) ||
        !FlashProgram_IsRangeValidForSlot(g_target_slot, addr, size)) {
        printf("[UDS-S] 0x34 addr/size out of range: 0x%08lX +%lu\r\n",
               (unsigned long)addr, (unsigned long)size);
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }

    {
        printf("[UDS-S] Committing UPDATE_IN_PROGRESS before App erase...\r\n");
        if (ImageMetadata_MarkUpdateInProgress(g_target_slot, size) !=
            IMAGE_METADATA_OK) {
            printf("[UDS-S] UPDATE_IN_PROGRESS commit FAIL; App preserved\r\n");
            send_negative_response(UDS_SID_REQ_DOWNLOAD,
                                   NRC_GENERAL_PROGRAMMING_FAILURE);
            return;
        }
    }
    
    /* UPDATE_IN_PROGRESS is durable before the destructive App erase. */
    printf("[UDS-S] Erasing authorized Slot %c (55 pages)...\r\n",
           g_target_slot == IMAGE_SLOT_A_ID ? 'A' : 'B');
    if (FlashProgram_EraseSlot(g_target_slot) != FLASH_PROGRAM_OK) {
        printf("[UDS-S] Erase FAIL\r\n");
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_GENERAL_PROGRAMMING_FAILURE);
        return;
    }
    printf("[UDS-S] Erase OK\r\n");
    
    g_download_addr = addr;
    g_download_size = size;
    g_download_received = 0;
    g_expected_block_seq = 1;
    g_state = UDS_SRV_DOWNLOAD_ACTIVE;
    
    printf("[UDS-S] 0x34 OK: addr=0x%08lX size=%lu\r\n",
           (unsigned long)addr, (unsigned long)size);
    
    g_resp_buf[0] = UDS_SID_REQ_DOWNLOAD + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = 0x20;
    g_resp_buf[2] = (MAX_BLOCK_SIZE >> 8) & 0xFF;
    g_resp_buf[3] = MAX_BLOCK_SIZE & 0xFF;
    
    send_positive_response(g_resp_buf, 4);
    printf("[UDS-S] -> 74 maxBlockSize=%u\r\n", MAX_BLOCK_SIZE);
}

/* ============ 0x36 TransferData ============ */
static void handle_transfer_data(const uint8_t *req, uint16_t len)
{
    if (g_state != UDS_SRV_DOWNLOAD_ACTIVE && g_state != UDS_SRV_TRANSFERRING) {
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_REQUEST_SEQUENCE_ERROR);
        return;
    }
    
    if (len < 2) {
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    uint8_t seq = req[1];
    uint16_t data_len = len - 2;
    
    if (data_len > MAX_BLOCK_SIZE) {
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    if (seq != g_expected_block_seq) {
        printf("[UDS-S] 0x36 seq mismatch: got %u, expected %u\r\n",
               seq, g_expected_block_seq);
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_REQUEST_SEQUENCE_ERROR);
        return;
    }
    
    if ((g_download_received > g_download_size) ||
        (data_len > (g_download_size - g_download_received))) {
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    /* Boot 端：直接写 Flash，不再 RAM 暂存 */
    /* 注意：data_len 必须偶数（halfword 对齐），如果固件是奇数字节，需要补位
       这里假设上位机已对齐 */
    if (data_len & 1) {
        printf("[UDS-S] 0x36 data_len %u not halfword aligned\r\n", data_len);
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_GENERAL_PROGRAMMING_FAILURE);
        return;
    }
    
    uint32_t target_addr = g_download_addr + g_download_received;
    if (FlashProgram_Write(target_addr, &req[2], data_len) !=
        FLASH_PROGRAM_OK) {
        printf("[UDS-S] 0x36 flash write FAIL @ 0x%08lX\r\n", (unsigned long)target_addr);
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_GENERAL_PROGRAMMING_FAILURE);
        return;
    }
    
    g_download_received += data_len;
    /* M8.1B1: ISO 14229 blockSequenceCounter uses the full uint8_t range.
       Do not skip 0x00: 0xFF -> 0x00 -> 0x01. */
    g_expected_block_seq = (uint8_t)(g_expected_block_seq + 1U);
    
    g_state = UDS_SRV_TRANSFERRING;
    
    g_resp_buf[0] = UDS_SID_TRANSFER_DATA + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = seq;
    send_positive_response(g_resp_buf, 2);
    
    printf("[UDS-S] 0x36 seq=%u +%u, total=%lu/%lu\r\n",
           seq, data_len,
           (unsigned long)g_download_received, (unsigned long)g_download_size);
}

/* ============ 0x37 RequestTransferExit ============ */
static void handle_transfer_exit(const uint8_t *req, uint16_t len)
{
    if (g_state != UDS_SRV_TRANSFERRING) {
        send_negative_response(UDS_SID_REQ_TRANSFER_EXIT, NRC_REQUEST_SEQUENCE_ERROR);
        return;
    }
    
    if (len != 1) {
        send_negative_response(UDS_SID_REQ_TRANSFER_EXIT, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    if (g_download_received != g_download_size) {
        printf("[UDS-S] 0x37 incomplete: %lu/%lu\r\n",
               (unsigned long)g_download_received, (unsigned long)g_download_size);
        send_negative_response(UDS_SID_REQ_TRANSFER_EXIT, NRC_CONDITIONS_NOT_CORRECT);
        return;
    }
    
    g_state = UDS_SRV_TRANSFER_DONE;
    
    g_resp_buf[0] = UDS_SID_REQ_TRANSFER_EXIT + UDS_POS_RESPONSE_OFFSET;
    send_positive_response(g_resp_buf, 1);
    printf("[UDS-S] 0x37 OK, total %lu bytes received\r\n",
           (unsigned long)g_download_received);
}

/* ============ 0x31 RoutineControl (CRC) ============ */
static void handle_routine_control(const uint8_t *req, uint16_t len)
{
    static const uint8_t unknown_version[4] = {0U, 0U, 0U, 0U};
    AppImageDescriptorType descriptor;
    ImageValidator_VectorType vector;
    const uint8_t *version_for_invalid = unknown_version;
    bool crc_ok = false;
    bool vector_ok = false;
    bool descriptor_ok = false;
    bool metadata_ok = false;
    uint32_t actual_crc = 0U;
    uint32_t status = 1U;

    if (len < 4) {
        send_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    uint8_t subfunc = req[1];
    uint16_t rid = ((uint16_t)req[2] << 8) | req[3];
    
    if (subfunc != UDS_ROUTINE_START) {
        send_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_SUB_FUNCTION_NOT_SUPPORTED);
        return;
    }
    
    if (rid != 0xF001) {
        send_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    if (len != 8) {
        send_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    if (g_state != UDS_SRV_TRANSFER_DONE) {
        send_negative_response(UDS_SID_ROUTINE_CONTROL, NRC_REQUEST_SEQUENCE_ERROR);
        return;
    }
    
    uint32_t expected_crc = ((uint32_t)req[4] << 24) | ((uint32_t)req[5] << 16) |
                            ((uint32_t)req[6] << 8)  | ((uint32_t)req[7]);
    
    if ((g_download_received == g_download_size) &&
        ImageValidator_IsImageRangeValidForSlot(g_target_slot,
                                                g_download_addr,
                                                g_download_size) &&
        FlashProgram_IsRangeValidForSlot(g_target_slot,
                                         g_download_addr,
                                         g_download_size)) {
        crc_ok = ImageValidator_VerifyCrc32(
            (const uint8_t *)g_download_addr,
            g_download_size,
            expected_crc,
            &actual_crc);
    }

    DemEventAdapter_ReportCrcResult(crc_ok, HAL_GetTick());

    if (crc_ok) {
        vector.initial_msp =
            *(volatile const uint32_t *)g_download_addr;
        vector.reset_handler =
            *(volatile const uint32_t *)(g_download_addr + 4U);
        vector_ok = ImageValidator_ValidateVectorForImage(
            vector.initial_msp,
            vector.reset_handler,
            g_download_addr,
            g_download_size);
    }

    if (vector_ok) {
        descriptor_ok = AppImageDescriptor_ReadAndValidate(
            g_download_addr,
            g_download_size,
            &descriptor);
    }

    if (descriptor_ok) {
        version_for_invalid = descriptor.software_version;
        metadata_ok = ImageMetadata_MarkPending(g_target_slot,
                                                g_download_size,
                                                expected_crc,
                                                descriptor.software_version) ==
                      IMAGE_METADATA_OK;
    }

    if (crc_ok && vector_ok && descriptor_ok && metadata_ok) {
        status = 0U;
        g_target_locked = false;
    } else {
        ImageMetadata_RollbackReasonType reason =
            !crc_ok ? IMAGE_ROLLBACK_PENDING_CRC_INVALID :
            !vector_ok ? IMAGE_ROLLBACK_PENDING_VECTOR_INVALID :
                         IMAGE_ROLLBACK_PENDING_DESCRIPTOR_INVALID;
        (void)version_for_invalid;
        (void)ImageMetadata_MarkCandidateInvalid(g_target_slot, reason);
    }

    DemEventAdapter_ReportAppValidity(vector_ok && descriptor_ok && metadata_ok,
                                      HAL_GetTick());
    /* A completed verify attempt (success or failure) ends this transaction. */
    g_target_locked = false;
    
    printf("[UDS-S] CRC expected=0x%08lX actual=0x%08lX %s\r\n",
           (unsigned long)expected_crc, (unsigned long)actual_crc,
           (status == 0U) ? "OK" : "FAIL");
    printf("[UDS-S] Verify vector=%u descriptor=%u metadata=%u\r\n",
           vector_ok ? 1U : 0U,
           descriptor_ok ? 1U : 0U,
           metadata_ok ? 1U : 0U);
    
    g_resp_buf[0] = UDS_SID_ROUTINE_CONTROL + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = UDS_ROUTINE_START;
    g_resp_buf[2] = 0xF0;
    g_resp_buf[3] = 0x01;
    g_resp_buf[4] = (status == 0U) ? 0x00 : 0x01;
    send_positive_response(g_resp_buf, 5);
}

/* ============ 0x11 ECUReset ============ */
static void handle_ecu_reset(const uint8_t *req, uint16_t len)
{
    if (len != 2) {
        send_negative_response(UDS_SID_ECU_RESET, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    uint8_t reset_type = req[1];
    
    if (reset_type != UDS_RESET_HARD &&
        reset_type != UDS_RESET_KEY_OFF_ON &&
        reset_type != UDS_RESET_SOFT) {
        send_negative_response(UDS_SID_ECU_RESET, NRC_SUB_FUNCTION_NOT_SUPPORTED);
        return;
    }
    
    g_resp_buf[0] = UDS_SID_ECU_RESET + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = reset_type;
    send_positive_response(g_resp_buf, 2);
    printf("[UDS-S] -> 51 %02X (Reset in 100ms, back to App)\r\n", reset_type);
    
    g_reset_pending = true;
    g_reset_tick = HAL_GetTick();
}

/* Boot 端 tick：复位时不写 magic（默认跳新 App）*/
void uds_server_tick(uint32_t now_ms)
{
    if (g_reset_pending &&
        ((uint32_t)(now_ms - g_reset_tick) >= 100U)) {
        printf("[UDS-S] *** REBOOT (back to App) ***\r\n");
        g_reset_pending = false;
        BootManager_ResetToApp();
    }
}

/* ============ 主调度入口 ============ */
void uds_server_on_request(const uint8_t *data, uint16_t len)
{
    if (len < 1) return;
    
    uint8_t sid = data[0];
    printf("[UDS-S] <- SID=0x%02X len=%u\r\n", sid, len);
    
    switch (sid) {
        case UDS_SID_DIAG_SESSION_CTRL:  handle_session_control(data, len);  break;
        case UDS_SID_ECU_RESET:          handle_ecu_reset(data, len);        break;
        case UDS_SID_READ_DATA_BY_ID:    handle_read_data_by_id(data, len);  break;
        case UDS_SID_SECURITY_ACCESS:    handle_security_access(data, len);  break;
        case UDS_SID_ROUTINE_CONTROL:    handle_routine_control(data, len);  break;
        case UDS_SID_REQ_DOWNLOAD:       handle_request_download(data, len); break;
        case UDS_SID_TRANSFER_DATA:      handle_transfer_data(data, len);    break;
        case UDS_SID_REQ_TRANSFER_EXIT:  handle_transfer_exit(data, len);    break;
        default:
            send_negative_response(sid, NRC_SERVICE_NOT_SUPPORTED);
            break;
    }
}

uds_server_state_t uds_server_get_state(void) { return g_state; }

void uds_server_init(void)
{
    g_state = UDS_SRV_DEFAULT_SESSION;
    g_seed_requested = false;
    g_download_received = 0;
    g_expected_block_seq = 1;
    g_reset_pending = false;
    g_target_locked = false;
    g_target_slot = IMAGE_SLOT_NONE;
    g_target_base = 0U;
    
    printf("[UDS-S] Boot UDS Server Init OK\r\n");
}

