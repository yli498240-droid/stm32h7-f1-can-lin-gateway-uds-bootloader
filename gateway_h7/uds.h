#ifndef __UDS_H__
#define __UDS_H__

#include <stdint.h>
#include <stdbool.h>

/* ===================== UDS 服务 ID ===================== */
#define UDS_SID_DIAG_SESSION_CTRL   0x10
#define UDS_SID_ECU_RESET           0x11
#define UDS_SID_READ_DATA_BY_ID     0x22
#define UDS_SID_SECURITY_ACCESS     0x27
#define UDS_SID_REQ_DOWNLOAD        0x34
#define UDS_SID_TRANSFER_DATA       0x36
#define UDS_SID_REQ_TRANSFER_EXIT   0x37
#define UDS_SID_ROUTINE_CONTROL     0x31

/* 肯定响应 = 服务ID + 0x40 */
#define UDS_POS_RESPONSE_OFFSET     0x40

/* 否定响应固定 SID */
#define UDS_NEG_RESPONSE            0x7F

/* ===================== 子功能 ===================== */
/* 0x10 子功能 */
#define UDS_SESSION_DEFAULT         0x01
#define UDS_SESSION_PROGRAMMING     0x02
#define UDS_SESSION_EXTENDED        0x03

/* 0x27 子功能 */
#define UDS_SEC_REQUEST_SEED        0x01
#define UDS_SEC_SEND_KEY            0x02

/* 0x11 子功能 */
#define UDS_RESET_HARD              0x01
#define UDS_RESET_SOFT              0x03

/* 0x31 子功能 */
#define UDS_ROUTINE_START           0x01
#define UDS_ROUTINE_STOP            0x02
#define UDS_ROUTINE_RESULT          0x03

/* ===================== NRC 否定响应码 ===================== */
#define UDS_NRC_GENERAL_REJECT              0x10
#define UDS_NRC_SERVICE_NOT_SUPPORTED       0x11
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED   0x12
#define UDS_NRC_INCORRECT_LENGTH            0x13
#define UDS_NRC_CONDITIONS_NOT_CORRECT      0x22
#define UDS_NRC_REQUEST_SEQUENCE_ERROR      0x24
#define UDS_NRC_SECURITY_ACCESS_DENIED      0x33
#define UDS_NRC_INVALID_KEY                 0x35
#define UDS_NRC_EXCEEDED_NUMBER_OF_ATTEMPTS 0x36

/* ===================== Tester 状态机 ===================== */
typedef enum {
    UDS_STATE_IDLE,
    UDS_STATE_DEFAULT_SESSION,
    UDS_STATE_PROGRAMMING_SESSION,
    UDS_STATE_SECURITY_UNLOCKED,
    UDS_STATE_DOWNLOAD_REQUESTED,
    UDS_STATE_TRANSFERRING,
    UDS_STATE_TRANSFER_DONE,
    UDS_STATE_VERIFY_OK,
} uds_tester_state_t;

/* ===================== 上层 API ===================== */

/**
 * @brief 初始化 UDS Tester
 */
void uds_tester_init(void);

/**
 * @brief 当 ISO-TP 重组完一段数据时被调用（应用层接收回调）
 */
void uds_on_response(const uint8_t *data, uint16_t len);

/**
 * @brief 启动一次完整的 OTA 刷写流程（在测试任务里调用）
 * @return true 全部成功，false 任意一步失败
 */
bool uds_run_flash_sequence(void);

/* 单服务调用 API（也可独立测试）*/
bool uds_diag_session_control(uint8_t session_type);
bool uds_ecu_reset(uint8_t reset_type);
bool uds_security_request_seed(uint32_t *out_seed);
bool uds_security_send_key(uint32_t key);
bool uds_read_ota_slot_target(uint8_t *slot_id,uint32_t *base);
bool uds_request_download(uint32_t addr, uint32_t size, uint16_t *max_block_size);
bool uds_transfer_data(uint8_t seq, const uint8_t *data, uint16_t len);
bool uds_request_transfer_exit(void);
bool uds_routine_control_crc(uint32_t crc, uint32_t *out_result);

#endif

