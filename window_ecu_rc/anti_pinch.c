#include "anti_pinch.h"

#include <stddef.h>
#include <stdio.h>

#define ANTIPINCH_SPEED_THRESHOLD          80
#define ANTIPINCH_COUNT_THRESHOLD          3U
#define ANTIPINCH_GRACE_MS                 500U

static uint8_t s_low_speed_count = 0U;

void AntiPinch_Init(void)
{
    AntiPinch_Reset();
}

void AntiPinch_Reset(void)
{
    s_low_speed_count = 0U;
}

AntiPinchResult_t AntiPinch_Update(const AntiPinchInput_t *input)
{
    AntiPinchResult_t result = ANTIPINCH_NO_TRIGGER;

    if (input == NULL) {
        return ANTIPINCH_NO_TRIGGER;
    }

    if (input->motion_elapsed_ms < ANTIPINCH_GRACE_MS) {
        s_low_speed_count = 0U;
        return ANTIPINCH_NO_TRIGGER;
    }

    if (input->encoder_delta < ANTIPINCH_SPEED_THRESHOLD) {
        if (s_low_speed_count < ANTIPINCH_COUNT_THRESHOLD) {
            s_low_speed_count++;
        }
        printf("[ANTIPINCH] LOW SPEED (delta=%ld, count=%u)\r\n",
               (long)input->encoder_delta, s_low_speed_count);
        if (s_low_speed_count >= ANTIPINCH_COUNT_THRESHOLD) {
            result = ANTIPINCH_TRIGGER_ENCODER;
        }
    } else {
        s_low_speed_count = 0U;
    }

    return result;
}
