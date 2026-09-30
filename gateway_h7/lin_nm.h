#ifndef LIN_NM_H
#define LIN_NM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIN_NM_AWAKE = 0,
    LIN_NM_GOING_TO_SLEEP,
    LIN_NM_SLEEP,
    LIN_NM_WAKEUP_PENDING
} LinNm_StateType;

void LinNm_Init(void);
bool LinNm_RequestSleep(void);
void LinNm_MainFunction(uint32_t now_ms);
LinNm_StateType LinNm_GetState(void);

#endif /* LIN_NM_H */
