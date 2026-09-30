#include "button.h"

#include <stddef.h>
#include <stdio.h>
#include "main.h"

#define BUTTON_LONG_THRESHOLD_MS  500U
#define BUTTON_DEBOUNCE_MS         30U
#define BUTTON_EVENT_QUEUE_CAPACITY  4U

typedef struct
{
    uint8_t pin_state_prev;
    uint32_t debounce_start_tick;
    uint32_t press_start_tick;
    uint8_t is_pressed;
    uint8_t long_fired;
} Button_State_t;

static Button_State_t s_button_open;
static Button_State_t s_button_close;
static ButtonEvent_t s_event_queue[BUTTON_EVENT_QUEUE_CAPACITY];
static uint8_t s_event_read;
static uint8_t s_event_write;
static uint8_t s_event_count;

static void Button_EnqueueEvent(ButtonEvent_t event)
{
    if (s_event_count < BUTTON_EVENT_QUEUE_CAPACITY) {
        s_event_queue[s_event_write] = event;
        s_event_write =
            (uint8_t)((s_event_write + 1U) % BUTTON_EVENT_QUEUE_CAPACITY);
        s_event_count++;
    }
}

static void Button_ScanOne(Button_State_t *state,
                           GPIO_TypeDef *port,
                           uint16_t pin,
                           ButtonEvent_t short_event,
                           ButtonEvent_t long_event,
                           const char *name,
                           uint32_t now_ms)
{
    uint8_t current =
        (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (current != state->pin_state_prev) {
        state->pin_state_prev = current;
        state->debounce_start_tick = now_ms;
    }

    if ((current != state->is_pressed) &&
        ((uint32_t)(now_ms - state->debounce_start_tick) >=
         BUTTON_DEBOUNCE_MS)) {
        state->is_pressed = current;

        if (current != 0U) {
            /* Start timing from the last raw edge that became stable. */
            state->press_start_tick = state->debounce_start_tick;
            state->long_fired = 0U;
            printf("[BTN] %s press\r\n", name);
        } else {
            if (state->long_fired == 0U) {
                printf("[BTN] %s short\r\n", name);
                Button_EnqueueEvent(short_event);
            } else {
                printf("[BTN] %s release_after_long\r\n", name);
                Button_EnqueueEvent(BUTTON_EVENT_LONG_RELEASE);
            }
        }
    }

    if ((state->is_pressed != 0U) &&
        (state->long_fired == 0U) &&
        ((uint32_t)(now_ms - state->press_start_tick) >=
         BUTTON_LONG_THRESHOLD_MS)) {
        state->long_fired = 1U;
        printf("[BTN] %s long\r\n", name);
        Button_EnqueueEvent(long_event);
    }
}

void Button_Init(void)
{
    s_button_open.pin_state_prev = 0U;
    s_button_open.debounce_start_tick = 0U;
    s_button_open.press_start_tick = 0U;
    s_button_open.is_pressed = 0U;
    s_button_open.long_fired = 0U;

    s_button_close.pin_state_prev = 0U;
    s_button_close.debounce_start_tick = 0U;
    s_button_close.press_start_tick = 0U;
    s_button_close.is_pressed = 0U;
    s_button_close.long_fired = 0U;

    s_event_read = 0U;
    s_event_write = 0U;
    s_event_count = 0U;
}

void Button_MainFunction(uint32_t now_ms)
{
    Button_ScanOne(&s_button_open, GPIOA, GPIO_PIN_15,
                   BUTTON_EVENT_SHORT_OPEN, BUTTON_EVENT_LONG_OPEN,
                   "OPEN", now_ms);
    Button_ScanOne(&s_button_close, GPIOA, GPIO_PIN_12,
                   BUTTON_EVENT_SHORT_CLOSE, BUTTON_EVENT_LONG_CLOSE,
                   "CLOSE", now_ms);
}

uint8_t Button_TryGetEvent(ButtonEvent_t *event)
{
    if ((event == NULL) || (s_event_count == 0U)) {
        return 0U;
    }

    *event = s_event_queue[s_event_read];
    s_event_read =
        (uint8_t)((s_event_read + 1U) % BUTTON_EVENT_QUEUE_CAPACITY);
    s_event_count--;
    return 1U;
}

void Button_ClearEvents(void)
{
    s_event_read = 0U;
    s_event_write = 0U;
    s_event_count = 0U;
}

uint8_t Button_GetPhysicalState(void)
{
    uint8_t state = 0U;

    if (s_button_open.is_pressed != 0U) state |= (1U << 0);
    if (s_button_close.is_pressed != 0U) state |= (1U << 1);
    if (((s_button_open.is_pressed != 0U) &&
         (s_button_open.long_fired != 0U)) ||
        ((s_button_close.is_pressed != 0U) &&
         (s_button_close.long_fired != 0U))) {
        state |= (1U << 2);
    }
    return state;
}
