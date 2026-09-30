#include "uds.h"
#include "diag_tester.h"
#include "rc_app_ota_image.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/* Public showcase placeholder; original lab demo key mask intentionally omitted. */
#define PUBLIC_SHOWCASE_KEY_XOR_MASK 0x00000000U

/* ============ 内部状态 ============ */
static uds_tester_state_t g_state = UDS_STATE_IDLE;

/* 等待响应机制：发送请求后挂起，收到响应或超时返回 */
#define UDS_RESPONSE_TIMEOUT_MS  10000
static volatile bool g_response_received = false;
static uint8_t  g_response_buf[256];
static uint16_t g_response_len = 0;

/* ============ ISO-TP 接收回调 ============ */
void uds_on_response(const uint8_t *data, uint16_t len)
{
    if (len > sizeof(g_response_buf)) len = sizeof(g_response_buf);
    memcpy(g_response_buf, data, len);
    g_response_len = len;
    g_response_received = true;
    
    /* 调试打印 */
    printf("[UDS RX] len=%u, SID=0x%02X", len, data[0]);
    if (data[0] == UDS_NEG_RESPONSE && len >= 3) {
        printf(" (NEG, ReqSID=0x%02X, NRC=0x%02X)", data[1], data[2]);
    }
    printf("\r\n");
}

/* ============ 同步等待响应（阻塞 API） ============ */
static bool wait_for_response(uint8_t expected_sid)
{
    g_response_received = false;
    
    uint32_t timeout = UDS_RESPONSE_TIMEOUT_MS;
    while (!g_response_received && timeout > 0) {
        osDelay(1);
        timeout--;
    }
    
    /* 不管什么情况，都打印一行诊断 */
    printf("[UDS-DBG] wait done: rcvd=%d, remain_to=%lu, len=%u\r\n",
           (int)g_response_received,
           (unsigned long)timeout,
           g_response_len);
    
    if (!g_response_received) {
        printf("[UDS] Timeout waiting for SID 0x%02X\r\n", expected_sid);
        return false;
    }
    
    if (g_response_len < 1) return false;
    if (g_response_buf[0] == UDS_NEG_RESPONSE) return false;
    if (g_response_buf[0] != (expected_sid + UDS_POS_RESPONSE_OFFSET)) {
        printf("[UDS] Unexpected SID: got 0x%02X, expected 0x%02X\r\n",
               g_response_buf[0], expected_sid + UDS_POS_RESPONSE_OFFSET);
        return false;
    }
    
    return true;
}

/* ============ 0x10 DiagnosticSessionControl ============ */
bool uds_diag_session_control(uint8_t session_type)
{
    uint8_t request[2] = { UDS_SID_DIAG_SESSION_CTRL, session_type };
    printf("[UDS TX] 0x10 SessionControl, session=0x%02X\r\n", session_type);
    
    DiagTester_StatusType ret = DiagTester_SendRequest(request, 2U);
    if (ret != DIAG_TESTER_OK) {
        printf("[UDS-DBG] DiagTester_SendRequest returned %d\r\n", ret);
        return false;
    }
    
    if (!wait_for_response(UDS_SID_DIAG_SESSION_CTRL))
        return false;
    
    /* === ISO 14229-1 响应格式 ===
     *   Byte 0:   0x50
     *   Byte 1:   diagnosticSessionType (echo, 1B)
     *   Byte 2-3: P2_server_max (big-endian, ms)
     *   Byte 4-5: P2*_server_max (big-endian, ×10ms)
     *   最小长度 = 6 字节
     */
    if (g_response_len < 6) {
        printf("[UDS] 0x10 response too short (len=%u, min=6)\r\n", g_response_len);
        return false;
    }
    
    /* 校验回显的 sessionType */
    if (g_response_buf[1] != session_type) {
        printf("[UDS] 0x10 session echo mismatch: got 0x%02X, expected 0x%02X\r\n",
               g_response_buf[1], session_type);
        return false;
    }
    
    /* 解析 P2/P2* 超时（仅作信息打印，不强制使用）*/
    uint16_t p2  = ((uint16_t)g_response_buf[2] << 8) | g_response_buf[3];
    uint16_t p2x = ((uint16_t)g_response_buf[4] << 8) | g_response_buf[5];
    printf("[UDS] Server timing: P2=%u ms, P2*=%u (×10ms)\r\n", p2, p2x);
    
    /* 切状态 */
    if (session_type == UDS_SESSION_PROGRAMMING) {
        g_state = UDS_STATE_PROGRAMMING_SESSION;
        printf("[UDS] State -> PROGRAMMING_SESSION\r\n");
    } else if (session_type == UDS_SESSION_DEFAULT) {
        g_state = UDS_STATE_DEFAULT_SESSION;
        printf("[UDS] State -> DEFAULT_SESSION\r\n");
    } else if (session_type == UDS_SESSION_EXTENDED) {
        printf("[UDS] State -> EXTENDED_SESSION\r\n");
    }
    
    return true;
}
/* ============ 0x11 ECUReset ============ */
bool uds_ecu_reset(uint8_t reset_type)
{
    uint8_t request[2] = { UDS_SID_ECU_RESET, reset_type };
    printf("[UDS TX] 0x11 ECUReset, type=0x%02X\r\n", reset_type);
    
    DiagTester_StatusType ret = DiagTester_SendRequest(request, 2U);
    if (ret != DIAG_TESTER_OK) return false;
    
    if (!wait_for_response(UDS_SID_ECU_RESET)) return false;
    
    /* === ISO 14229-1 响应格式 ===
     *   Byte 0:   0x51
     *   Byte 1:   resetType (echo, 1B)
     *   Byte 2:   powerDownTime (可选)
     *   最小长度 = 2 字节
     */
    if (g_response_len < 2) {
        printf("[UDS] 0x11 response too short (len=%u, min=2)\r\n", g_response_len);
        return false;
    }
    
    if (g_response_buf[1] != reset_type) {
        printf("[UDS] 0x11 reset type echo mismatch: got 0x%02X, expected 0x%02X\r\n",
               g_response_buf[1], reset_type);
        return false;
    }
    
    g_state = UDS_STATE_IDLE;
    printf("[UDS] State -> IDLE (reset complete)\r\n");
    return true;
}

/* ============ 0x27 SecurityAccess - Request Seed ============ */
bool uds_security_request_seed(uint32_t *out_seed)
{
    uint8_t request[2] = { UDS_SID_SECURITY_ACCESS, UDS_SEC_REQUEST_SEED };
    printf("[UDS TX] 0x27 RequestSeed\r\n");
    
    DiagTester_StatusType ret = DiagTester_SendRequest(request, 2U);
    if (ret != DIAG_TESTER_OK) return false;
    
    if (!wait_for_response(UDS_SID_SECURITY_ACCESS)) return false;
    
    /* === ISO 14229-1 响应格式 ===
     *   Byte 0:   0x67
     *   Byte 1:   securityAccessType (echo)
     *   Byte 2+:  securitySeed (本实现为 4 字节)
     *   最小长度 = 6 字节（含 4B Seed）
     */
    if (g_response_len < 6) {
        printf("[UDS] 0x27 RequestSeed response too short (len=%u, min=6)\r\n",
               g_response_len);
        return false;
    }
    
    /* 校验子功能回显 */
    if (g_response_buf[1] != UDS_SEC_REQUEST_SEED) {
        printf("[UDS] 0x27 subfunc echo mismatch: got 0x%02X, expected 0x%02X\r\n",
               g_response_buf[1], UDS_SEC_REQUEST_SEED);
        return false;
    }
    
    /* 大端解析 4 字节 Seed */
    *out_seed = ((uint32_t)g_response_buf[2] << 24) |
                ((uint32_t)g_response_buf[3] << 16) |
                ((uint32_t)g_response_buf[4] << 8)  |
                ((uint32_t)g_response_buf[5]);
    
    /* 检查 Seed 是否全 0（已解锁状态下 Server 会返回全 0 Seed）*/
    if (*out_seed == 0) {
        printf("[UDS] Seed is zero (already unlocked)\r\n");
    }
    
    printf("[UDS] Got Seed = 0x%08lX\r\n", (unsigned long)*out_seed);
    return true;
}

/* ============ 0x27 SecurityAccess - Send Key ============ */
bool uds_security_send_key(uint32_t key)
{
    uint8_t request[6];
    request[0] = UDS_SID_SECURITY_ACCESS;
    request[1] = UDS_SEC_SEND_KEY;
    request[2] = (key >> 24) & 0xFF;
    request[3] = (key >> 16) & 0xFF;
    request[4] = (key >> 8)  & 0xFF;
    request[5] = key & 0xFF;
    
    printf("[UDS TX] 0x27 SendKey = 0x%08lX\r\n", (unsigned long)key);
    
    DiagTester_StatusType ret = DiagTester_SendRequest(request, 6U);
    if (ret != DIAG_TESTER_OK) return false;
    
    if (!wait_for_response(UDS_SID_SECURITY_ACCESS)) return false;
    
    /* === ISO 14229-1 响应格式 ===
     *   Byte 0:   0x67
     *   Byte 1:   securityAccessType (echo, = 0x02)
     *   最小长度 = 2 字节
     */
    if (g_response_len < 2) {
        printf("[UDS] 0x27 SendKey response too short (len=%u, min=2)\r\n",
               g_response_len);
        return false;
    }
    
    if (g_response_buf[1] != UDS_SEC_SEND_KEY) {
        printf("[UDS] 0x27 SendKey subfunc echo mismatch: got 0x%02X\r\n",
               g_response_buf[1]);
        return false;
    }
    
    g_state = UDS_STATE_SECURITY_UNLOCKED;
    printf("[UDS] State -> SECURITY_UNLOCKED\r\n");
    return true;
}

/* ============ 0x22 F1A0 Boot-authorized OTA target ============ */
bool uds_read_ota_slot_target(uint8_t *slot_id,uint32_t *base)
{
    uint8_t request[3] = {UDS_SID_READ_DATA_BY_ID, 0xF1U, 0xA0U};
    uint16_t page_index;
    if ((slot_id == NULL) || (base == NULL)) return false;
    printf("[UDS TX] 0x22 F1A0 query Boot target\r\n");
    if (DiagTester_SendRequest(request, 3U) != DIAG_TESTER_OK) return false;
    if (!wait_for_response(UDS_SID_READ_DATA_BY_ID)) return false;
    if ((g_response_len != 6U) || (g_response_buf[1] != 0xF1U) ||
        (g_response_buf[2] != 0xA0U) ||
        ((g_response_buf[3] != RC_APP_OTA_SLOT_A) &&
         (g_response_buf[3] != RC_APP_OTA_SLOT_B))) return false;
    page_index = ((uint16_t)g_response_buf[4] << 8U) | g_response_buf[5];
    *slot_id = g_response_buf[3];
    *base = 0x08000000UL + ((uint32_t)page_index * 0x800UL);
    if (((*slot_id == RC_APP_OTA_SLOT_A) && (*base != RC_APP_OTA_SLOT_A_BASE)) ||
        ((*slot_id == RC_APP_OTA_SLOT_B) && (*base != RC_APP_OTA_SLOT_B_BASE))) return false;
    printf("[UDS] Boot target Slot %c @0x%08lX\r\n",
           *slot_id == RC_APP_OTA_SLOT_A ? 'A' : 'B',(unsigned long)*base);
    return true;
}

/* ============ 0x34 RequestDownload ============ */
bool uds_request_download(uint32_t addr, uint32_t size, uint16_t *max_block_size)
{
    /* 请求格式：34 + dataFormatId + addrLengthFormatId + addr(4B) + size(4B) */
    uint8_t request[11];
    request[0] = UDS_SID_REQ_DOWNLOAD;
    request[1] = 0x00;  /* dataFormatId: 无加密无压缩 */
    request[2] = 0x44;  /* addrLengthFormatId: 高4位=size长度4B, 低4位=addr长度4B */
    request[3] = (addr >> 24) & 0xFF;
    request[4] = (addr >> 16) & 0xFF;
    request[5] = (addr >> 8)  & 0xFF;
    request[6] = addr & 0xFF;
    request[7]  = (size >> 24) & 0xFF;
    request[8]  = (size >> 16) & 0xFF;
    request[9]  = (size >> 8)  & 0xFF;
    request[10] = size & 0xFF;
    
    printf("[UDS TX] 0x34 RequestDownload, addr=0x%08lX, size=%lu\r\n",
           (unsigned long)addr, (unsigned long)size);
    
    DiagTester_StatusType ret = DiagTester_SendRequest(request, 11U);
    if (ret != DIAG_TESTER_OK) return false;
    
    if (!wait_for_response(UDS_SID_REQ_DOWNLOAD)) return false;
    
    /* === ISO 14229-1 响应格式 ===
     *   Byte 0:   0x74
     *   Byte 1:   lengthFormatIdentifier (高4位 = blockSize 字节数, 低4位保留)
     *   Byte 2+:  maxNumberOfBlockLength (大端，N 字节)
     *   最小长度 = 3（LFI 高 4 位 = 1，blockSize 1B）
     */
    if (g_response_len < 3) {
        printf("[UDS] 0x34 response too short (len=%u, min=3)\r\n", g_response_len);
        return false;
    }
    
    /* 解析 LengthFormatIdentifier 高 4 位 */
    uint8_t lfi = (g_response_buf[1] >> 4) & 0x0F;
    if (lfi == 0 || lfi > 4) {
        printf("[UDS] 0x34 invalid LFI: 0x%02X\r\n", g_response_buf[1]);
        return false;
    }
    
    /* 校验响应总长是否够装下 LFI 指示的 blockSize */
    if (g_response_len < (uint16_t)(2 + lfi)) {
        printf("[UDS] 0x34 LFI=%u but only %u bytes payload\r\n", lfi, g_response_len);
        return false;
    }
    
    /* 大端解析 maxNumberOfBlockLength（按 LFI 长度）*/
    uint32_t mbs = 0;
    for (uint8_t i = 0; i < lfi; i++) {
        mbs = (mbs << 8) | g_response_buf[2 + i];
    }
    
    if (mbs == 0 || mbs > 0xFFFF) {
        printf("[UDS] 0x34 invalid maxBlockSize=%lu\r\n", (unsigned long)mbs);
        return false;
    }
    
    *max_block_size = (uint16_t)mbs;
    printf("[UDS] Got max_block_size = %u (LFI=%u)\r\n", *max_block_size, lfi);
    
    g_state = UDS_STATE_DOWNLOAD_REQUESTED;
    return true;
}

/* ============ 0x36 TransferData ============ */
bool uds_transfer_data(uint8_t seq, const uint8_t *data, uint16_t len)
{
    if (len > 250) {
        printf("[UDS] 0x36 block too big: %u\r\n", len);
        return false;
    }
    
    static uint8_t request[256];
    request[0] = UDS_SID_TRANSFER_DATA;
    request[1] = seq;  /* Block Sequence Counter */
    memcpy(&request[2], data, len);
    
    printf("[UDS TX] 0x36 TransferData, seq=%u, len=%u\r\n", seq, len);
    
    DiagTester_StatusType ret = DiagTester_SendRequest(request, (uint16_t)(2U + len));
    if (ret != DIAG_TESTER_OK) return false;
    
    if (!wait_for_response(UDS_SID_TRANSFER_DATA)) return false;
    
    /* === ISO 14229-1 响应格式 ===
     *   Byte 0:   0x76
     *   Byte 1:   blockSequenceCounter (echo)
     *   Byte 2+:  transferResponseParameterRecord (可选)
     *   最小长度 = 2 字节
     */
    if (g_response_len < 2) {
        printf("[UDS] 0x36 response too short (len=%u, min=2)\r\n", g_response_len);
        return false;
    }
    
    if (g_response_buf[1] != seq) {
        printf("[UDS] 0x36 seq mismatch: got %u, expected %u\r\n",
               g_response_buf[1], seq);
        return false;
    }
    
    g_state = UDS_STATE_TRANSFERRING;
    return true;
}

/* ============ 0x37 RequestTransferExit ============ */
bool uds_request_transfer_exit(void)
{
    uint8_t request[1] = { UDS_SID_REQ_TRANSFER_EXIT };
    printf("[UDS TX] 0x37 RequestTransferExit\r\n");
    
    DiagTester_StatusType ret = DiagTester_SendRequest(request, 1U);
    if (ret != DIAG_TESTER_OK) return false;
    
    if (!wait_for_response(UDS_SID_REQ_TRANSFER_EXIT)) return false;
    
    /* === ISO 14229-1 响应格式 ===
     *   Byte 0:   0x77
     *   Byte 1+:  transferResponseParameterRecord (可选)
     *   最小长度 = 1 字节
     */
    if (g_response_len < 1) {
        printf("[UDS] 0x37 response too short (len=%u, min=1)\r\n", g_response_len);
        return false;
    }
    
    g_state = UDS_STATE_TRANSFER_DONE;
    return true;
}

/* ============ 0x31 RoutineControl - CRC Check ============ */
/* ============ 0x31 RoutineControl - CRC Check ============ */
bool uds_routine_control_crc(uint32_t crc, uint32_t *out_result)
{
    uint8_t request[8];
    request[0] = UDS_SID_ROUTINE_CONTROL;
    request[1] = UDS_ROUTINE_START;       /* startRoutine */
    request[2] = 0xF0;                     /* RoutineID 高字节 */
    request[3] = 0x01;                     /* RoutineID 低字节: 0xF001 = CRC check */
    /* CRC 大端 */
    request[4] = (crc >> 24) & 0xFF;
    request[5] = (crc >> 16) & 0xFF;
    request[6] = (crc >> 8)  & 0xFF;
    request[7] = crc & 0xFF;
    
    printf("[UDS TX] 0x31 RoutineControl CRC=0x%08lX\r\n", (unsigned long)crc);
    
    DiagTester_StatusType ret = DiagTester_SendRequest(request, 8U);
    if (ret != DIAG_TESTER_OK) return false;
    
    if (!wait_for_response(UDS_SID_ROUTINE_CONTROL)) return false;
    
    /* === 响应格式（ISO 14229-1）===
     *   Byte 0:   0x71 (positive response SID)
     *   Byte 1:   routineControlType (0x01 = start)
     *   Byte 2-3: routineIdentifier (2 bytes)
     *   Byte 4+:  routineStatusRecord (可选变长)
     *
     * 最小合法长度 = 4（不带 statusRecord）
     */
    if (g_response_len < 4) {
        printf("[UDS] 0x31 response too short (len=%u, min=4)\r\n", g_response_len);
        return false;
    }
    
    /* 校验子功能 */
    if (g_response_buf[1] != UDS_ROUTINE_START) {
        printf("[UDS] 0x31 unexpected subfunc: 0x%02X\r\n", g_response_buf[1]);
        return false;
    }
    
    /* 校验 RoutineID */
    uint16_t rid = ((uint16_t)g_response_buf[2] << 8) | g_response_buf[3];
    if (rid != 0xF001) {
        printf("[UDS] 0x31 unexpected RID: 0x%04X\r\n", rid);
        return false;
    }
    
    /* 解析 statusRecord（可选）*/
    if (g_response_len >= 8) {
        /* 含 4 字节 statusRecord（典型场景）*/
        *out_result = ((uint32_t)g_response_buf[4] << 24) |
                      ((uint32_t)g_response_buf[5] << 16) |
                      ((uint32_t)g_response_buf[6] << 8)  |
                      ((uint32_t)g_response_buf[7]);
        printf("[UDS] CRC check result = 0x%08lX (4-byte status)\r\n",
               (unsigned long)*out_result);
    } else if (g_response_len == 5) {
        /* 含 1 字节状态码 */
        *out_result = g_response_buf[4];
        printf("[UDS] CRC check result = 0x%02X (1-byte status)\r\n",
               (unsigned int)*out_result);
    } else {
        /* 无 statusRecord，仅启动确认 */
        *out_result = 0x00000000;
        printf("[UDS] CRC routine started (no status record)\r\n");
    }
    
    if (*out_result != 0U) {
        printf("[UDS] CRC status indicates failure; reset is inhibited\r\n");
        return false;
    }

    g_state = UDS_STATE_VERIFY_OK;
    return true;
}

/* ============ Seed → Key 算法（必须和 Server 端一致） ============ */
static uint32_t calculate_key_from_seed(uint32_t seed)
{
    uint32_t key = (~seed) ^ PUBLIC_SHOWCASE_KEY_XOR_MASK;
    printf("[UDS] Seed=0x%08lX -> Key=0x%08lX\r\n",
           (unsigned long)seed, (unsigned long)key);
    return key;
}

/* ============ Real RC App OTA image CRC32 ============ */
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

static bool artifact_preflight(const OtaArtifact_Type *artifact,
                               uint8_t target_slot,uint32_t target_base)
{
    uint32_t descriptor_base;
    if ((artifact == NULL) || (artifact->data == NULL) ||
        (artifact->size < 0x220U) || (artifact->slot_id != target_slot) ||
        (artifact->base != target_base)) return false;
    descriptor_base = (uint32_t)artifact->data[0x20CU] |
        ((uint32_t)artifact->data[0x20DU] << 8U) |
        ((uint32_t)artifact->data[0x20EU] << 16U) |
        ((uint32_t)artifact->data[0x20FU] << 24U);
    if (descriptor_base != artifact->base) return false;
    return crc32_calc(artifact->data,artifact->size) == artifact->crc32;
}

/* ============ 先把 F103 踢进 Boot 模式 ============ */
/* App 收到 0x11 后会写 magic word + 复位，进入 Boot OTA 模式 */
bool uds_kick_to_boot(void)
{
    printf("[UDS] === Step 0: Kick F103 into Boot mode ===\r\n");
    
    if (!uds_ecu_reset(UDS_RESET_SOFT)) {
        printf("[UDS] Failed to send 0x11\r\n");
        return false;
    }
    
    /* 等 F103 复位 + Boot 启动 + 进入 OTA 模式 */
    printf("[UDS] Waiting 3s for F103 to enter Boot OTA mode...\r\n");
    osDelay(3000);
    
    printf("[UDS] F103 should be in Boot now\r\n");
    return true;
}

/* ============ 完整刷写序列（先做最简版，只调用 0x10 和 0x11） ============ */
bool uds_run_flash_sequence(void)
{
    OtaArtifact_Type artifact;
    uint8_t target_slot;
    uint32_t target_base;
    printf("\r\n========== UDS Flash Sequence Start ==========\r\n");
    
    /* Step 0: 先踢 F103 进 Boot 模式 */
    if (!uds_kick_to_boot()) {
        printf("[UDS] FAILED at kick-to-boot\r\n"); return false;
    }
    
    /* 1. 进入编程会话 */
    if (!uds_diag_session_control(UDS_SESSION_PROGRAMMING)) {
        printf("[UDS] FAILED at 0x10\r\n"); return false;
    }
    
    /* 2. 请求 Seed */
    uint32_t seed;
    if (!uds_security_request_seed(&seed)) {
        printf("[UDS] FAILED at 0x27 (RequestSeed)\r\n"); return false;
    }
    
    /* 3. 计算 Key 并发送 */
    uint32_t key = (~seed) ^ PUBLIC_SHOWCASE_KEY_XOR_MASK;
    if (!uds_security_send_key(key)) {
        printf("[UDS] FAILED at 0x27 (SendKey)\r\n"); return false;
    }

    /* Boot is the sole target authority. Fail before 0x34 on any mismatch. */
    if (!uds_read_ota_slot_target(&target_slot,&target_base) ||
        !OtaArtifact_GetForSlot(target_slot,&artifact) ||
        !artifact_preflight(&artifact,target_slot,target_base)) {
        printf("[UDS] Target/artifact preflight failed; 0x34 inhibited\r\n");
        return false;
    }
    
    /* 4. 请求下载 */
    uint16_t max_block_size;
    if (!uds_request_download(artifact.base,
                              artifact.size,
                              &max_block_size)) {
        printf("[UDS] FAILED at 0x34\r\n"); return false;
    }
    
    /* 5. Transfer the complete real App image in fixed 32-byte data blocks. */
		const uint16_t block_size = RC_APP_OTA_BLOCK_SIZE;
		if (max_block_size < block_size) {
				printf("[UDS] Server max block size %u is below required %u\r\n",
				       max_block_size, block_size);
				return false;
		}
		uint8_t seq = 1;
		for (uint32_t offset = 0; offset < artifact.size; offset += block_size) {
        uint32_t remaining = artifact.size - offset;
        uint16_t chunk = (uint16_t)remaining;
        if (chunk > block_size) chunk = block_size;
        
        if (!uds_transfer_data(seq, &artifact.data[offset], chunk)) {
            printf("[UDS] FAILED at 0x36 seq=%u\r\n", seq); return false;
        }
        seq++;
    }
    
    /* 6. 结束传输 */
    if (!uds_request_transfer_exit()) {
        printf("[UDS] FAILED at 0x37\r\n"); return false;
    }
    
    /* 7. CRC 校验 */
		uint32_t crc_start_tick = osKernelGetTickCount();
		uint32_t expected_crc = crc32_calc(artifact.data, artifact.size);
		uint32_t crc_elapsed = osKernelGetTickCount() - crc_start_tick;
		printf("[UDS] Expected CRC = 0x%08lX (calc took %lu ms)\r\n",
					 (unsigned long)expected_crc, (unsigned long)crc_elapsed);

    if (expected_crc != artifact.crc32) {
        printf("[UDS] Embedded image CRC metadata mismatch; download aborted\r\n");
        return false;
    }
    
    uint32_t crc_result;
    if (!uds_routine_control_crc(expected_crc, &crc_result)) {
        printf("[UDS] FAILED at 0x31\r\n"); return false;
    }
    
    /* 8. 重启 ECU */
    if (!uds_ecu_reset(UDS_RESET_SOFT)) {
        printf("[UDS] FAILED at 0x11\r\n"); return false;
    }
    
    printf("========== UDS Flash Sequence COMPLETE ==========\r\n\r\n");
    return true;
}

/* ============ 初始化 ============ */
void uds_tester_init(void)
{
    g_state = UDS_STATE_IDLE;
    g_response_received = false;
    g_response_len = 0;
    
    printf("[UDS] Tester Init OK\r\n");
    printf("[UDS] Artifact A: %u bytes, %u blocks; B: %u bytes, %u blocks\r\n",
           (unsigned int)g_rc_app_ota_slot_a_image_length,
           (unsigned int)((g_rc_app_ota_slot_a_image_length +
                           RC_APP_OTA_BLOCK_SIZE - 1U) /
                          RC_APP_OTA_BLOCK_SIZE),
           (unsigned int)g_rc_app_ota_slot_b_image_length,
           (unsigned int)((g_rc_app_ota_slot_b_image_length +
                           RC_APP_OTA_BLOCK_SIZE - 1U) /
                          RC_APP_OTA_BLOCK_SIZE));
}

