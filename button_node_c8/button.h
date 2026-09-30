#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

typedef enum
{
    BUTTON_EVENT_NONE         = 0x00,
    BUTTON_EVENT_SHORT_OPEN   = 0x01,
    BUTTON_EVENT_SHORT_CLOSE  = 0x02,
    BUTTON_EVENT_LONG_OPEN    = 0x03,
    BUTTON_EVENT_LONG_CLOSE   = 0x04,
    BUTTON_EVENT_LONG_RELEASE = 0x05
} ButtonEvent_t;

void Button_Init(void);
void Button_MainFunction(uint32_t now_ms);
uint8_t Button_TryGetEvent(ButtonEvent_t *event);
void Button_ClearEvents(void);
/* bit0=OPEN pressed, bit1=CLOSE pressed, bit2=long-press active */
uint8_t Button_GetPhysicalState(void);

#endif /* BUTTON_H */
