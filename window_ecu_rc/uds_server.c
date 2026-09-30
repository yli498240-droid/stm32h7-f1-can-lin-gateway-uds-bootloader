#include "uds_server.h"
#include "dcm.h"
#include "dcm_dem_if.h"
#include "main.h"
#include "app_version.h"
#include "window_control.h"
#include "reset_reason.h"
#include <string.h>
#include <stdio.h>

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
#define APP_AREA_START          0x08008000U
#define APP_AREA_END_EXCLUSIVE  0x0803F000U
#define MAX_BLOCK_SIZE   512

static uint32_t g_download_addr = 0;
static uint32_t g_download_size = 0;
static uint32_t g_download_received = 0;
static uint8_t  g_expected_block_seq = 1;

/* 接收缓冲区（暂存固件，Phase 4 不写 Flash，Phase 5 才写）*/
#define DOWNLOAD_BUF_SIZE   2048
static uint8_t  g_download_buf[DOWNLOAD_BUF_SIZE];

/* Pending Reset：响应发出去之后再实际复位 */
static volatile bool g_reset_pending = false;
static volatile uint32_t g_reset_tick = 0;

/* ============ 辅助函数 ============ */
static void send_negative_response(uint8_t req_sid, uint8_t nrc)
{
    g_resp_buf[0] = UDS_NEG_RESPONSE;
    g_resp_buf[1] = req_sid;
    g_resp_buf[2] = nrc;
    printf("[UDS-S] -> NEG: SID=0x%02X NRC=0x%02X\r\n", req_sid, nrc);
    Dcm_SendResponse(g_resp_buf, 3U);
}

static void send_positive_response(const uint8_t *data, uint16_t len)
{
    Dcm_SendResponse(data, len);
}

/* ============ 0x22 ReadDataByIdentifier ============ */
static void handle_read_data_by_identifier(const uint8_t *req, uint16_t len)
{
    uint16_t did;
    int32_t encoder_position;
    uint32_t encoder_raw;

    /* M10.2 deliberately supports exactly one DID per request. */
    if (len != 3U) {
        send_negative_response(UDS_SID_READ_DATA_BY_IDENTIFIER,
                               NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }

    did = (uint16_t)(((uint16_t)req[1] << 8) | req[2]);
    g_resp_buf[0] = UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = req[1];
    g_resp_buf[2] = req[2];

    switch (did) {
        case UDS_DID_SOFTWARE_VERSION:
            g_resp_buf[3] = (uint8_t)RC_APP_VERSION_MAJOR;
            g_resp_buf[4] = (uint8_t)RC_APP_VERSION_MINOR;
            g_resp_buf[5] = (uint8_t)RC_APP_VERSION_PATCH;
            g_resp_buf[6] = (uint8_t)RC_APP_VERSION_BUILD;
            send_positive_response(g_resp_buf, 7U);
            break;

        case UDS_DID_ECU_APPLICATION_ID:
            memcpy(&g_resp_buf[3], RC_APP_APPLICATION_ID,
                   RC_APP_APPLICATION_ID_LENGTH);
            send_positive_response(g_resp_buf,
                                   3U + RC_APP_APPLICATION_ID_LENGTH);
            break;

        case UDS_DID_WINDOW_RUNTIME_STATUS:
            encoder_position = WindowControl_GetEncoderPosition();
            encoder_raw = (uint32_t)encoder_position;
            g_resp_buf[3] = WindowControl_GetDirection();
            g_resp_buf[4] = (uint8_t)(encoder_raw >> 24);
            g_resp_buf[5] = (uint8_t)(encoder_raw >> 16);
            g_resp_buf[6] = (uint8_t)(encoder_raw >> 8);
            g_resp_buf[7] = (uint8_t)encoder_raw;
            send_positive_response(g_resp_buf, 8U);
            break;

        case UDS_DID_RESET_REASON:
        {
            uint32_t flags = ResetReason_GetRawFlags();
            g_resp_buf[3] = (uint8_t)ResetReason_Get();
            g_resp_buf[4] = (uint8_t)(flags >> 24);
            g_resp_buf[5] = (uint8_t)(flags >> 16);
            g_resp_buf[6] = (uint8_t)(flags >> 8);
            g_resp_buf[7] = (uint8_t)flags;
            send_positive_response(g_resp_buf, 8U);
            break;
        }

        default:
            send_negative_response(UDS_SID_READ_DATA_BY_IDENTIFIER,
                                   NRC_REQUEST_OUT_OF_RANGE);
            break;
    }
}

/* ============ 0x19 ReadDTCInformation ============ */
static void handle_read_dtc_information(const uint8_t *req, uint16_t len)
{
    uint8_t subfunction;
    uint8_t status_mask;
    uint16_t dtc_count;
    DcmDemIf_ReturnType dem_result;

    if (len != 3U) {
        send_negative_response(UDS_SID_READ_DTC_INFORMATION,
                               NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }

    subfunction = req[1];
    status_mask = req[2];

    if ((subfunction != UDS_READ_DTC_NUMBER_BY_STATUS_MASK) &&
        (subfunction != UDS_READ_DTC_BY_STATUS_MASK)) {
        send_negative_response(UDS_SID_READ_DTC_INFORMATION,
                               NRC_SUB_FUNCTION_NOT_SUPPORTED);
        return;
    }

    dem_result = DcmDemIf_GetDtcCountByStatusMask(status_mask, &dtc_count);
    if (dem_result != DCM_DEM_E_OK) {
        send_negative_response(UDS_SID_READ_DTC_INFORMATION,
                               NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    if (subfunction == UDS_READ_DTC_NUMBER_BY_STATUS_MASK) {
        g_resp_buf[0] = UDS_SID_READ_DTC_INFORMATION + UDS_POS_RESPONSE_OFFSET;
        g_resp_buf[1] = UDS_READ_DTC_NUMBER_BY_STATUS_MASK;
        g_resp_buf[2] = DCM_DEM_DTC_STATUS_AVAILABILITY_MASK;
        g_resp_buf[3] = DCM_DEM_DTC_FORMAT_IDENTIFIER;
        g_resp_buf[4] = (uint8_t)(dtc_count >> 8);
        g_resp_buf[5] = (uint8_t)dtc_count;
        send_positive_response(g_resp_buf, 6U);
        return;
    }

    if (dtc_count > ((sizeof(g_resp_buf) - 3U) / 4U)) {
        send_negative_response(UDS_SID_READ_DTC_INFORMATION,
                               NRC_CONDITIONS_NOT_CORRECT);
        return;
    }

    g_resp_buf[0] = UDS_SID_READ_DTC_INFORMATION + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = UDS_READ_DTC_BY_STATUS_MASK;
    g_resp_buf[2] = DCM_DEM_DTC_STATUS_AVAILABILITY_MASK;

    for (uint16_t index = 0U; index < dtc_count; index++) {
        DcmDemIf_DtcRecordType dtc_record;
        uint16_t response_offset = (uint16_t)(3U + (index * 4U));

        dem_result = DcmDemIf_GetDtcByStatusMaskIndex(status_mask,
                                                       index,
                                                       &dtc_record);
        if (dem_result != DCM_DEM_E_OK) {
            send_negative_response(UDS_SID_READ_DTC_INFORMATION,
                                   NRC_CONDITIONS_NOT_CORRECT);
            return;
        }

        g_resp_buf[response_offset] = (uint8_t)(dtc_record.dtc >> 16);
        g_resp_buf[response_offset + 1U] = (uint8_t)(dtc_record.dtc >> 8);
        g_resp_buf[response_offset + 2U] = (uint8_t)dtc_record.dtc;
        g_resp_buf[response_offset + 3U] = dtc_record.status;
    }

    send_positive_response(g_resp_buf, (uint16_t)(3U + (dtc_count * 4U)));
}

/* ============ 0x14 ClearDiagnosticInformation ============ */
static void handle_clear_diagnostic_information(const uint8_t *req,
                                                uint16_t len)
{
    if (len != 4U) {
        send_negative_response(UDS_SID_CLEAR_DIAGNOSTIC_INFO,
                               NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }

    if ((req[1] != 0xFFU) || (req[2] != 0xFFU) || (req[3] != 0xFFU)) {
        send_negative_response(UDS_SID_CLEAR_DIAGNOSTIC_INFO,
                               NRC_REQUEST_OUT_OF_RANGE);
        return;
    }

    DcmDemIf_ClearAllDtc();
    g_resp_buf[0] = UDS_SID_CLEAR_DIAGNOSTIC_INFO + UDS_POS_RESPONSE_OFFSET;
    send_positive_response(g_resp_buf, 1U);
}

/* CRC32（多项式 0xEDB88320，跟 H7 端一致）*/
static uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else         crc >>= 1;
        }
    }
    return ~crc;
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
    
    /* 切会话时清安全状态、清 download 状态 */
    g_seed_requested = false;
    g_download_received = 0;
    
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
    g_resp_buf[2] = 0x00; g_resp_buf[3] = 0x32;   /* P2 = 50ms */
    g_resp_buf[4] = 0x01; g_resp_buf[5] = 0xF4;   /* P2* = 5000 ×10ms */
    
    send_positive_response(g_resp_buf, 6);
    printf("[UDS-S] -> 50 %02X (Session OK)\r\n", session_type);
}

/* ============ 0x27 SecurityAccess ============ */
static void handle_security_access(const uint8_t *req, uint16_t len)
{
    if (len < 2) {
        send_negative_response(UDS_SID_SECURITY_ACCESS, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    /* 必须在 Programming Session 下 */
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
        
        /* 已解锁状态返回全 0 Seed（标准做法）*/
        if (g_state == UDS_SRV_SECURITY_UNLOCKED ||
            g_state == UDS_SRV_DOWNLOAD_ACTIVE ||
            g_state == UDS_SRV_TRANSFERRING ||
            g_state == UDS_SRV_TRANSFER_DONE) {
            g_seed = 0;
				} else {
						/* 调试期固定 Seed，方便手算 Key（量产前改回随机）*/
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
        
        /* 算法：必须和 H7 Tester 一致 */
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
    /* 必须已解锁 */
    if (g_state != UDS_SRV_SECURITY_UNLOCKED) {
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_SECURITY_ACCESS_DENIED);
        return;
    }
    
    /* 最短 11 字节：SID + dataFormat + addrLenFmt + addr(4) + size(4) */
    if (len != 11) {
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_INCORRECT_MESSAGE_LENGTH);
        return;
    }
    
    uint8_t alfid = req[2];
    /* 只接受 addr=4B size=4B 的格式（高 4 位 size，低 4 位 addr）*/
    if (alfid != 0x44) {
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    uint32_t addr = ((uint32_t)req[3] << 24) | ((uint32_t)req[4] << 16) |
                    ((uint32_t)req[5] << 8)  | ((uint32_t)req[6]);
    uint32_t size = ((uint32_t)req[7] << 24) | ((uint32_t)req[8] << 16) |
                    ((uint32_t)req[9] << 8)  | ((uint32_t)req[10]);
    
    /* 地址范围校验 */
    if ((addr < APP_AREA_START) ||
        (addr >= APP_AREA_END_EXCLUSIVE) ||
        (size == 0U) ||
        (size > (APP_AREA_END_EXCLUSIVE - addr))) {
        printf("[UDS-S] 0x34 addr/size out of range: 0x%08lX +%lu\r\n",
               (unsigned long)addr, (unsigned long)size);
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    /* Phase 4 阶段：要求 size <= 我们的接收缓冲区大小 */
    if (size > DOWNLOAD_BUF_SIZE) {
        printf("[UDS-S] 0x34 size %lu > buf %u (Phase4 limit)\r\n",
               (unsigned long)size, DOWNLOAD_BUF_SIZE);
        send_negative_response(UDS_SID_REQ_DOWNLOAD, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    g_download_addr = addr;
    g_download_size = size;
    g_download_received = 0;
    g_expected_block_seq = 1;
    g_state = UDS_SRV_DOWNLOAD_ACTIVE;
    
    printf("[UDS-S] 0x34 OK: addr=0x%08lX size=%lu\r\n",
           (unsigned long)addr, (unsigned long)size);
    
    /* 响应：74 LFI maxBlockSize(2B) */
    g_resp_buf[0] = UDS_SID_REQ_DOWNLOAD + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = 0x20;   /* LFI: 高 4 位 = 2 表示 maxBlockSize 用 2 字节 */
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
    
    /* 校验序列号 */
    if (seq != g_expected_block_seq) {
        printf("[UDS-S] 0x36 seq mismatch: got %u, expected %u\r\n",
               seq, g_expected_block_seq);
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_REQUEST_SEQUENCE_ERROR);
        return;
    }
    
    /* 检查是否会越界 */
    if (g_download_received + data_len > g_download_size) {
        send_negative_response(UDS_SID_TRANSFER_DATA, NRC_REQUEST_OUT_OF_RANGE);
        return;
    }
    
    /* 拷贝到接收缓冲区（Phase 4 不写 Flash，Phase 5 改成写 Flash）*/
    memcpy(&g_download_buf[g_download_received], &req[2], data_len);
    g_download_received += data_len;
    g_expected_block_seq = (g_expected_block_seq + 1) & 0xFF;
    if (g_expected_block_seq == 0) g_expected_block_seq = 1;  /* 0 不用 */
    
    g_state = UDS_SRV_TRANSFERRING;
    
    /* 正响应：76 seq */
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
    
    /* 检查是否收完 */
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
    /* 最短 4 字节：SID + subfunc + RoutineID(2) */
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
    
    /* 0xF001 = CRC 校验，需要 4 字节 expected CRC */
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
    
    uint32_t actual_crc = crc32_calc(g_download_buf, g_download_received);
    uint32_t status = (actual_crc == expected_crc) ? 0 : 1;
    
    printf("[UDS-S] CRC expected=0x%08lX actual=0x%08lX %s\r\n",
           (unsigned long)expected_crc, (unsigned long)actual_crc,
           (status == 0) ? "OK" : "FAIL");
    
    /* 正响应：71 01 F0 01 status[4] */
		g_resp_buf[0] = UDS_SID_ROUTINE_CONTROL + UDS_POS_RESPONSE_OFFSET;
		g_resp_buf[1] = UDS_ROUTINE_START;
		g_resp_buf[2] = 0xF0;
		g_resp_buf[3] = 0x01;
		g_resp_buf[4] = (status == 0) ? 0x00 : 0x01;  /* 1B status */

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
    
    /* 先回响应 */
    g_resp_buf[0] = UDS_SID_ECU_RESET + UDS_POS_RESPONSE_OFFSET;
    g_resp_buf[1] = reset_type;
    send_positive_response(g_resp_buf, 2);
    printf("[UDS-S] -> 51 %02X (Reset in 100ms)\r\n", reset_type);
    
    /* 排队 100ms 后真复位（让响应有时间送出去）*/
    g_reset_pending = true;
    g_reset_tick = HAL_GetTick();
}

/* 主循环里轮询此函数，到时执行真复位 */
void uds_server_tick(void)
{
    if (g_reset_pending && (HAL_GetTick() - g_reset_tick >= 100)) {
        printf("[UDS-S] *** REBOOT (request boot mode) ***\r\n");
        HAL_Delay(50);
        
        /* 写 magic word 让 Boot 复位后停在 Boot 模式 */
        *((volatile uint32_t*)0x2000BFFC) = 0xDEADBEEF;
        
        NVIC_SystemReset();
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
        case UDS_SID_CLEAR_DIAGNOSTIC_INFO:
            handle_clear_diagnostic_information(data, len);
            break;
        case UDS_SID_READ_DTC_INFORMATION:
            handle_read_dtc_information(data, len);
            break;
        case UDS_SID_READ_DATA_BY_IDENTIFIER:
            handle_read_data_by_identifier(data, len);
            break;
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
    
    printf("[UDS-S] Server Init OK, State=DEFAULT_SESSION\r\n");
}

