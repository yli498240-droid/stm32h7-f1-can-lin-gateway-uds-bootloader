#include "open_stall_protection.h"

#include <stdio.h>

#define OPEN_STALL_SPEED_THRESHOLD  50
#define OPEN_STALL_COUNT_THRESHOLD  20
#define OPEN_STALL_GRACE_MS         500U

static uint8_t s_low_speed_count = 0U;

void OpenStallProtection_Init(void)
{
    OpenStallProtection_Reset();
}

void OpenStallProtection_Reset(void)
{
    s_low_speed_count = 0U;
}

uint8_t OpenStallProtection_Update(int32_t encoder_delta,
                                  uint32_t motion_elapsed_ms)
{
    if (motion_elapsed_ms < OPEN_STALL_GRACE_MS) {
        s_low_speed_count = 0U;
        return 0U;
    }

    if (encoder_delta < OPEN_STALL_SPEED_THRESHOLD) {
        s_low_speed_count++;
        if ((s_low_speed_count == 5U) ||
            (s_low_speed_count == 10U) ||
            (s_low_speed_count == 15U)) {
            printf("[OVERLOAD] OPEN stalled (delta=%ld count=%u)\r\n",
                   (long)encoder_delta, s_low_speed_count);
        }
        if (s_low_speed_count >= OPEN_STALL_COUNT_THRESHOLD) {
            return 1U;
        }
    } else {
        s_low_speed_count = 0U;
    }

    return 0U;
}
