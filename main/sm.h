#pragma once

#include <stdbool.h>

typedef enum {
    SM_STATE_IDLE = 0,
    SM_STATE_READY,
    SM_STATE_CALIBRATE,
    SM_STATE_MAPPING,
    SM_STATE_RACING,
    SM_STATE_MANUAL,
} sm_state_t;

typedef enum {
    SM_EVENT_NONE = 0,
    SM_EVENT_ARM,
    SM_EVENT_START_CALIBRATION,
    SM_EVENT_CALIBRATION_DONE,
    SM_EVENT_START_MAPPING,
    SM_EVENT_START_RACING,
    SM_EVENT_ENTER_MANUAL,
    SM_EVENT_STOP,
} sm_event_t;

typedef struct {
    sm_state_t state;
} sm_t;

void sm_init(sm_t *sm);
sm_state_t sm_get_state(const sm_t *sm);
bool sm_handle_event(sm_t *sm, sm_event_t event);
void sm_run_current_state(const sm_t *sm);
const char *sm_state_name(sm_state_t state);
const char *sm_event_name(sm_event_t event);
