#include "button_event.h"

#include <stddef.h>

#define BUTTON_EVENT_SHORT_OPEN    0x01U
#define BUTTON_EVENT_SHORT_CLOSE   0x02U
#define BUTTON_EVENT_LONG_OPEN     0x03U
#define BUTTON_EVENT_LONG_CLOSE    0x04U
#define BUTTON_EVENT_LONG_RELEASE  0x05U

#define WINDOW_COMMAND_AUTO_OPEN   0x01U
#define WINDOW_COMMAND_AUTO_CLOSE  0x02U
#define WINDOW_COMMAND_JOG_OPEN    0x04U
#define WINDOW_COMMAND_JOG_CLOSE   0x05U

#define MOTION_HINT_OPEN           0x01U
#define MOTION_HINT_CLOSE          0x02U
#define BUTTON_EVENT_QUEUE_CAPACITY  8U

static volatile uint8_t s_event_queue[BUTTON_EVENT_QUEUE_CAPACITY];
static volatile uint8_t s_event_write_index = 0U;
static volatile uint8_t s_event_read_index = 0U;
static volatile uint8_t s_last_accepted_sequence = 0U;
static volatile uint8_t s_ack_sequence = 0U;
static volatile uint32_t s_queue_drop_count = 0U;
static volatile uint32_t s_duplicate_count = 0U;

void ButtonEvent_QueueInit(void)
{
    s_event_write_index = 0U;
    s_event_read_index = 0U;
    s_last_accepted_sequence = 0U;
    s_ack_sequence = 0U;
    s_queue_drop_count = 0U;
    s_duplicate_count = 0U;
}

uint8_t ButtonEvent_OfferFromLin(uint8_t event, uint8_t sequence)
{
    uint8_t next_index;

    if ((event < 0x01U) || (event > 0x05U) || (sequence == 0U)) {
        return 0U;
    }

    if (sequence == s_last_accepted_sequence) {
        s_duplicate_count++;
        s_ack_sequence = sequence;
        return 1U;
    }

    next_index = (uint8_t)((s_event_write_index + 1U) %
                           BUTTON_EVENT_QUEUE_CAPACITY);
    if (next_index == s_event_read_index) {
        s_queue_drop_count++;
        return 0U;
    }

    s_event_queue[s_event_write_index] = event;
    s_event_write_index = next_index;
    s_last_accepted_sequence = sequence;
    s_ack_sequence = sequence;
    return 1U;
}

uint8_t ButtonEvent_TryGetQueued(uint8_t *event)
{
    uint8_t read_index = s_event_read_index;

    if ((event == NULL) || (read_index == s_event_write_index)) {
        return 0U;
    }
    *event = s_event_queue[read_index];
    s_event_read_index =
        (uint8_t)((read_index + 1U) % BUTTON_EVENT_QUEUE_CAPACITY);
    return 1U;
}

uint8_t ButtonEvent_HasPending(void)
{
    return (s_event_read_index != s_event_write_index) ? 1U : 0U;
}

uint8_t ButtonEvent_GetAckSequence(void)
{
    return s_ack_sequence;
}

uint32_t ButtonEvent_GetQueueDropCount(void)
{
    return s_queue_drop_count;
}

uint32_t ButtonEvent_GetDuplicateCount(void)
{
    return s_duplicate_count;
}

uint8_t ButtonEvent_Translate(uint8_t event, ButtonEvent_Action_t *action)
{
    if (action == NULL) {
        return 0U;
    }

    action->action = BUTTON_EVENT_ACTION_IGNORE;
    action->command = 0U;
    action->update_motion_hint = 0U;
    action->motion_hint = 0U;

    switch (event) {
        case BUTTON_EVENT_SHORT_OPEN:
            action->action = BUTTON_EVENT_ACTION_SEND_COMMAND;
            action->command = WINDOW_COMMAND_JOG_OPEN;
            action->update_motion_hint = 1U;
            action->motion_hint = MOTION_HINT_OPEN;
            break;

        case BUTTON_EVENT_SHORT_CLOSE:
            action->action = BUTTON_EVENT_ACTION_SEND_COMMAND;
            action->command = WINDOW_COMMAND_JOG_CLOSE;
            action->update_motion_hint = 1U;
            action->motion_hint = MOTION_HINT_CLOSE;
            break;

        case BUTTON_EVENT_LONG_OPEN:
            action->action = BUTTON_EVENT_ACTION_SEND_COMMAND;
            action->command = WINDOW_COMMAND_AUTO_OPEN;
            action->update_motion_hint = 1U;
            action->motion_hint = MOTION_HINT_OPEN;
            break;

        case BUTTON_EVENT_LONG_CLOSE:
            action->action = BUTTON_EVENT_ACTION_SEND_COMMAND;
            action->command = WINDOW_COMMAND_AUTO_CLOSE;
            action->update_motion_hint = 1U;
            action->motion_hint = MOTION_HINT_CLOSE;
            break;

        case BUTTON_EVENT_LONG_RELEASE:
            /* AUTO continues: no CAN command and no motion-hint update. */
            break;

        default:
            return 0U;
    }

    return 1U;
}

const char *ButtonEvent_GetName(uint8_t event)
{
    switch (event) {
        case BUTTON_EVENT_SHORT_OPEN:
            return "SHORT_OPEN -> JOG_OPEN";
        case BUTTON_EVENT_SHORT_CLOSE:
            return "SHORT_CLOSE -> JOG_CLOSE";
        case BUTTON_EVENT_LONG_OPEN:
            return "LONG_OPEN -> OPEN HOLD";
        case BUTTON_EVENT_LONG_CLOSE:
            return "LONG_CLOSE -> CLOSE HOLD";
        case BUTTON_EVENT_LONG_RELEASE:
            return "LONG_RELEASE";
        default:
            return "?";
    }
}
