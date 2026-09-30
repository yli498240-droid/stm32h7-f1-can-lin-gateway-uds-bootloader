#ifndef __UDS_SERVER_H
#define __UDS_SERVER_H

#include <stdint.h>
#include <stdbool.h>

/* ============ UDS SID ============ */
#define UDS_SID_DIAG_SESSION_CTRL   0x10
#define UDS_SID_ECU_RESET            0x11
#define UDS_SID_CLEAR_DIAGNOSTIC_INFO 0x14
#define UDS_SID_READ_DTC_INFORMATION  0x19
#define UDS_SID_READ_DATA_BY_IDENTIFIER 0x22
#define UDS_SID_SECURITY_ACCESS      0x27
#define UDS_SID_ROUTINE_CONTROL      0x31
#define UDS_SID_REQ_DOWNLOAD         0x34
#define UDS_SID_TRANSFER_DATA        0x36
#define UDS_SID_REQ_TRANSFER_EXIT    0x37

#define UDS_POS_RESPONSE_OFFSET      0x40   /* 正响应 SID = 请求 SID + 0x40 */
#define UDS_NEG_RESPONSE             0x7F

/* ============ Session Types ============ */
#define UDS_SESSION_DEFAULT          0x01
#define UDS_SESSION_PROGRAMMING      0x02
#define UDS_SESSION_EXTENDED         0x03

/* ============ Security Subfunctions ============ */
#define UDS_SEC_REQUEST_SEED         0x01
#define UDS_SEC_SEND_KEY             0x02

/* ============ Routine Subfunctions ============ */
#define UDS_ROUTINE_START            0x01
#define UDS_ROUTINE_STOP             0x02
#define UDS_ROUTINE_RESULT           0x03

/* ============ Reset Types ============ */
#define UDS_RESET_HARD               0x01
#define UDS_RESET_KEY_OFF_ON         0x02
#define UDS_RESET_SOFT               0x03

/* ============ ReadDTCInformation Subfunctions ============ */
#define UDS_READ_DTC_NUMBER_BY_STATUS_MASK  0x01
#define UDS_READ_DTC_BY_STATUS_MASK         0x02

/* ============ ReadDataByIdentifier ============ */
#define UDS_DID_SOFTWARE_VERSION            0xF100U
#define UDS_DID_ECU_APPLICATION_ID          0xF101U
#define UDS_DID_WINDOW_RUNTIME_STATUS       0xF102U
#define UDS_DID_RESET_REASON                0xF103U

/* ============ NRC Codes ============ */
#define NRC_GENERAL_REJECT                       0x10
#define NRC_SERVICE_NOT_SUPPORTED                0x11
#define NRC_SUB_FUNCTION_NOT_SUPPORTED           0x12
#define NRC_INCORRECT_MESSAGE_LENGTH             0x13
#define NRC_CONDITIONS_NOT_CORRECT               0x22
#define NRC_REQUEST_SEQUENCE_ERROR               0x24
#define NRC_REQUEST_OUT_OF_RANGE                 0x31
#define NRC_SECURITY_ACCESS_DENIED               0x33
#define NRC_INVALID_KEY                          0x35
#define NRC_SERVICE_NOT_SUPPORTED_IN_SESSION     0x7F

/* ============ Server 状态 ============ */
typedef enum {
    UDS_SRV_DEFAULT_SESSION,
    UDS_SRV_EXTENDED_SESSION,
    UDS_SRV_PROGRAMMING_SESSION,
    UDS_SRV_SECURITY_UNLOCKED,      /* 在 Programming 下解锁后进入 */
    UDS_SRV_DOWNLOAD_ACTIVE,        /* 收到 0x34 后 */
    UDS_SRV_TRANSFERRING,           /* 收到第一个 0x36 后 */
    UDS_SRV_TRANSFER_DONE,          /* 收到 0x37 后 */
} uds_server_state_t;

/* ============ API ============ */
void uds_server_init(void);
void uds_server_tick(void);   /* 主循环里调用，处理延迟复位 */
void uds_server_on_request(const uint8_t *data, uint16_t len);  /* ISO-TP 收到请求时调用 */
uds_server_state_t uds_server_get_state(void);

#endif

