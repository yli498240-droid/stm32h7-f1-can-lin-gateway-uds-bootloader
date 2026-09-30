#include "window_control.h"

#include <stdio.h>

#include "main.h"
#include "anti_pinch.h"
#include "dem_event_adapter.h"
#include "encoder_adapter.h"
#include "motor_control.h"
#include "open_stall_protection.h"

#define WINDOW_RUN_PWM            300U
#define ANTIPINCH_REVERSE_PWM      700U
#define JOG_DURATION_MS            200U
#define CONTROL_SAMPLE_PERIOD_MS   50U
#define ANTIPINCH_REVERSE_TIME_MS  500U
#define ENC_TRAVEL_LIMIT           30000

static WindowControl_Direction_t s_direction = WINDOW_DIR_IDLE;
static int32_t s_encoder_position = 0;
static uint8_t s_jog_mode = 0U;
static uint32_t s_jog_start_tick = 0U;
static uint8_t s_auto_mode = 0U;
static int32_t s_auto_start_enc = 0;
static uint32_t s_motion_start_tick = 0U;
static uint32_t s_antipinch_start = 0U;
static uint16_t s_pwm_duty = 0U;
static uint32_t s_last_check_tick = 0U;
static int32_t s_last_enc = 0;

static void WindowControl_ResetAntiPinch(void)
{
    AntiPinch_Reset();
}

void WindowControl_Init(void)
{
    MotorControl_Init();
    printf("[MOTOR] Init done, BRAKE\r\n");

    EncoderAdapter_Init();
    printf("[ENC] Encoder started\r\n");

    AntiPinch_Init();
    OpenStallProtection_Init();
    printf("[ANTIPINCH] Encoder-only protection active\r\n");

    s_direction = WINDOW_DIR_IDLE;
    s_encoder_position = 0;
    s_jog_mode = 0U;
    s_jog_start_tick = 0U;
    s_auto_mode = 0U;
    s_auto_start_enc = 0;
    s_motion_start_tick = 0U;
    s_antipinch_start = 0U;
    s_pwm_duty = 0U;
    s_last_check_tick = 0U;
    s_last_enc = 0;
    MotorControl_Brake();
}

void WindowControl_HandleCommand(uint8_t command, uint32_t now_ms)
{
    if ((s_direction == WINDOW_DIR_ANTIPINCH_DONE) &&
        (command != WINDOW_CMD_RESET)) {
        printf("[CMD] LOCKED, RESET required\r\n");
        return;
    }

    switch (command) {
    case WINDOW_CMD_STOP:
        MotorControl_Brake();
        s_pwm_duty = 0U;
        s_direction = WINDOW_DIR_IDLE;
        s_jog_mode = 0U;
        s_auto_mode = 0U;
        WindowControl_ResetAntiPinch();
        printf("[CMD] STOP\r\n");
        break;

    case WINDOW_CMD_OPEN:
        if ((s_direction == WINDOW_DIR_CLOSING) &&
            (s_auto_mode || s_jog_mode)) {
            MotorControl_Brake();
            s_pwm_duty = 0U;
            s_direction = WINDOW_DIR_IDLE;
            s_auto_mode = 0U;
            s_jog_mode = 0U;
            WindowControl_ResetAntiPinch();
            printf("[CMD] AUTO_OPEN -> reverse cancel, STOP\r\n");
            break;
        }
        MotorControl_Open(WINDOW_RUN_PWM);
        s_pwm_duty = WINDOW_RUN_PWM;
        s_direction = WINDOW_DIR_OPENING;
        s_motion_start_tick = now_ms;
        s_jog_mode = 0U;
        s_auto_mode = 1U;
        s_auto_start_enc = EncoderAdapter_GetPosition();
        WindowControl_ResetAntiPinch();
        printf("[CMD] AUTO OPEN (continuous, will stop at travel limit)\r\n");
        break;

    case WINDOW_CMD_CLOSE:
        if ((s_direction == WINDOW_DIR_OPENING) &&
            (s_auto_mode || s_jog_mode)) {
            MotorControl_Brake();
            s_pwm_duty = 0U;
            s_direction = WINDOW_DIR_IDLE;
            s_auto_mode = 0U;
            s_jog_mode = 0U;
            WindowControl_ResetAntiPinch();
            printf("[CMD] AUTO_CLOSE -> reverse cancel, STOP\r\n");
            break;
        }
        MotorControl_Close(WINDOW_RUN_PWM);
        s_pwm_duty = WINDOW_RUN_PWM;
        s_direction = WINDOW_DIR_CLOSING;
        s_motion_start_tick = now_ms;
        s_jog_mode = 0U;
        s_auto_mode = 1U;
        s_auto_start_enc = EncoderAdapter_GetPosition();
        WindowControl_ResetAntiPinch();
        printf("[CMD] AUTO CLOSE (continuous, antipinch ON, stop at limit)\r\n");
        break;

    case WINDOW_CMD_RESET:
        MotorControl_Brake();
        s_pwm_duty = 0U;
        s_direction = WINDOW_DIR_IDLE;
        s_jog_mode = 0U;
        s_auto_mode = 0U;
        WindowControl_ResetAntiPinch();
        OpenStallProtection_Reset();
        printf("[CMD] RESET (back to IDLE)\r\n");
        DemEventAdapter_NotifyAntiPinchRecovered(now_ms);
        DemEventAdapter_NotifyOpenStallRecovered(now_ms);
        break;

    case WINDOW_CMD_JOG_OPEN:
        MotorControl_Open(WINDOW_RUN_PWM);
        s_pwm_duty = WINDOW_RUN_PWM;
        s_direction = WINDOW_DIR_OPENING;
        s_motion_start_tick = now_ms;
        s_jog_mode = 1U;
        s_auto_mode = 0U;
        s_jog_start_tick = now_ms;
        WindowControl_ResetAntiPinch();
        printf("[CMD] JOG OPEN (200ms pulse)\r\n");
        break;

    case WINDOW_CMD_JOG_CLOSE:
        MotorControl_Close(WINDOW_RUN_PWM);
        s_pwm_duty = WINDOW_RUN_PWM;
        s_direction = WINDOW_DIR_CLOSING;
        s_motion_start_tick = now_ms;
        s_jog_mode = 1U;
        s_auto_mode = 0U;
        s_jog_start_tick = now_ms;
        WindowControl_ResetAntiPinch();
        printf("[CMD] JOG CLOSE (200ms pulse)\r\n");
        break;

    default:
        printf("[CMD] Unknown 0x%02X\r\n", command);
        break;
    }
}

void WindowControl_MainFunction(uint32_t now_ms)
{
    if (s_jog_mode &&
        (now_ms - s_jog_start_tick >= JOG_DURATION_MS)) {
        MotorControl_Brake();
        s_pwm_duty = 0U;
        s_direction = WINDOW_DIR_IDLE;
        s_jog_mode = 0U;
        s_auto_mode = 0U;
        WindowControl_ResetAntiPinch();
        printf("[JOG] auto stop\r\n");
    }

    if (s_auto_mode &&
        ((s_direction == WINDOW_DIR_OPENING) ||
         (s_direction == WINDOW_DIR_CLOSING))) {
        int32_t enc_now = EncoderAdapter_GetPosition();
        int32_t travel = enc_now - s_auto_start_enc;
        if (travel < 0) {
            travel = -travel;
        }
        if (travel >= ENC_TRAVEL_LIMIT) {
            MotorControl_Brake();
            s_pwm_duty = 0U;
            s_direction = WINDOW_DIR_IDLE;
            s_auto_mode = 0U;
            WindowControl_ResetAntiPinch();
            printf("[AUTO] travel limit reached (%ld), stop\r\n",
                   (long)travel);
        }
    }

    if (now_ms - s_last_check_tick >= CONTROL_SAMPLE_PERIOD_MS) {
        int32_t enc = EncoderAdapter_GetPosition();
        int32_t delta = enc - s_last_enc;
        if (delta < 0) {
            delta = -delta;
        }

        s_last_check_tick = now_ms;
        s_last_enc = enc;
        s_encoder_position = enc;

        if (s_direction == WINDOW_DIR_ANTIPINCH_REV) {
            if (now_ms - s_antipinch_start >=
                ANTIPINCH_REVERSE_TIME_MS) {
                MotorControl_Brake();
                s_pwm_duty = 0U;
                s_direction = WINDOW_DIR_ANTIPINCH_DONE;
                printf("[ANTIPINCH] Done, waiting RESET\r\n");
            }
        } else if (s_direction == WINDOW_DIR_OPENING) {
            if (OpenStallProtection_Update(
                    delta, now_ms - s_motion_start_tick)) {
                MotorControl_Brake();
                s_pwm_duty = 0U;
                s_direction = WINDOW_DIR_IDLE;
                s_auto_mode = 0U;
                DemEventAdapter_NotifyOpenStallTriggered(now_ms);
                OpenStallProtection_Reset();
                DemEventAdapter_NotifyOpenStallRecovered(now_ms);
                printf("[OVERLOAD] *** OPEN motor PROTECT *** stop\r\n");
            }
        } else if (s_direction == WINDOW_DIR_CLOSING) {
            AntiPinchInput_t input;
            AntiPinchResult_t result;

            input.encoder_delta = delta;
            input.motion_elapsed_ms = now_ms - s_motion_start_tick;

            result = AntiPinch_Update(&input);
            if (result == ANTIPINCH_TRIGGER_ENCODER) {
                printf("[ANTIPINCH] Encoder trigger delta=%ld\r\n",
                       (long)input.encoder_delta);

                MotorControl_Reverse(ANTIPINCH_REVERSE_PWM);
                s_pwm_duty = ANTIPINCH_REVERSE_PWM;
                s_direction = WINDOW_DIR_ANTIPINCH_REV;
                s_antipinch_start = now_ms;
                s_jog_mode = 0U;
                AntiPinch_Reset();
                printf("[ANTIPINCH] Reversing 500ms\r\n");
                DemEventAdapter_NotifyAntiPinchTriggered(now_ms);
            }
        }
    }
}

uint8_t WindowControl_GetDirection(void)
{
    return (uint8_t)s_direction;
}

int32_t WindowControl_GetEncoderPosition(void)
{
    return s_encoder_position;
}
