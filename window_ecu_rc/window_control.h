#ifndef WINDOW_CONTROL_H
#define WINDOW_CONTROL_H

#include <stdint.h>

typedef enum {
    WINDOW_CMD_STOP      = 0x00,
    WINDOW_CMD_OPEN      = 0x01,
    WINDOW_CMD_CLOSE     = 0x02,
    WINDOW_CMD_RESET     = 0x03,
    WINDOW_CMD_JOG_OPEN  = 0x04,
    WINDOW_CMD_JOG_CLOSE = 0x05
} WindowControl_Command_t;

typedef enum {
    WINDOW_DIR_IDLE          = 0,
    WINDOW_DIR_OPENING       = 1,
    WINDOW_DIR_CLOSING       = 2,
    WINDOW_DIR_ANTIPINCH_REV = 3,
    WINDOW_DIR_ANTIPINCH_DONE = 4
} WindowControl_Direction_t;

void WindowControl_Init(void);
void WindowControl_HandleCommand(uint8_t command, uint32_t now_ms);
void WindowControl_MainFunction(uint32_t now_ms);
uint8_t WindowControl_GetDirection(void);
int32_t WindowControl_GetEncoderPosition(void);

#endif /* WINDOW_CONTROL_H */
