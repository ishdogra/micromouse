#include "sm.h"

#include "esp_log.h"

static const char *TAG = "state_machine";

void sm_init(sm_t *sm)
{
    if (sm == NULL) {
        return;
    }

    sm->state = SM_STATE_IDLE;
}

sm_state_t sm_get_state(const sm_t *sm)
{
    if (sm == NULL) {
        return SM_STATE_IDLE;
    }

    return sm->state;
}

bool sm_handle_event(sm_t *sm, sm_event_t event)
{
    if (sm == NULL) {
        return false;
    }

    const sm_state_t old_state = sm->state;
    sm_state_t next_state = old_state;

    switch (old_state) {
    case SM_STATE_IDLE:
        if (event == SM_EVENT_ARM) {
            next_state = SM_STATE_READY;
        } else if (event == SM_EVENT_ENTER_MANUAL) {
            next_state = SM_STATE_MANUAL;
        }
        break;

    case SM_STATE_READY:
        if (event == SM_EVENT_START_CALIBRATION) {
            next_state = SM_STATE_CALIBRATE;
        } else if (event == SM_EVENT_START_MAPPING) {
            next_state = SM_STATE_MAPPING;
        } else if (event == SM_EVENT_START_RACING) {
            next_state = SM_STATE_RACING;
        } else if (event == SM_EVENT_ENTER_MANUAL) {
            next_state = SM_STATE_MANUAL;
        } else if (event == SM_EVENT_STOP) {
            next_state = SM_STATE_IDLE;
        }
        break;

    case SM_STATE_CALIBRATE:
        if (event == SM_EVENT_CALIBRATION_DONE) {
            next_state = SM_STATE_READY;
        } else if (event == SM_EVENT_STOP) {
            next_state = SM_STATE_IDLE;
        }
        break;

    case SM_STATE_MAPPING:
    case SM_STATE_RACING:
    case SM_STATE_MANUAL:
        if (event == SM_EVENT_STOP) {
            next_state = SM_STATE_IDLE;
        }
        break;

    default:
        next_state = SM_STATE_IDLE;
        break;
    }

    if (next_state == old_state) {
        ESP_LOGW(TAG, "Ignored event %s while in %s",
                 sm_event_name(event), sm_state_name(old_state));
        return false;
    }

    sm->state = next_state;
    ESP_LOGI(TAG, "%s -> %s on %s",
             sm_state_name(old_state), sm_state_name(next_state), sm_event_name(event));
    return true;
}

void sm_run_current_state(const sm_t *sm)
{
    if (sm == NULL) {
        return;
    }

    switch (sm->state) {
    case SM_STATE_IDLE:
        ESP_LOGI(TAG, "Idle: motors off, waiting for command");
        break;
    case SM_STATE_READY:
        ESP_LOGI(TAG, "Ready: sensors active, waiting for mode");
        break;
    case SM_STATE_CALIBRATE:
        ESP_LOGI(TAG, "Calibrate: run IMU/sensor/motor calibration");
        break;
    case SM_STATE_MAPPING:
        ESP_LOGI(TAG, "Mapping: explore maze and update map");
        break;
    case SM_STATE_RACING:
        ESP_LOGI(TAG, "Racing: follow fastest known path");
        break;
    case SM_STATE_MANUAL:
        ESP_LOGI(TAG, "Manual: accept direct user control");
        break;
    default:
        ESP_LOGW(TAG, "Unknown state");
        break;
    }
}

const char *sm_state_name(sm_state_t state)
{
    switch (state) {
    case SM_STATE_IDLE:
        return "idle";
    case SM_STATE_READY:
        return "ready";
    case SM_STATE_CALIBRATE:
        return "calibrate";
    case SM_STATE_MAPPING:
        return "mapping";
    case SM_STATE_RACING:
        return "racing";
    case SM_STATE_MANUAL:
        return "manual";
    default:
        return "unknown";
    }
}

const char *sm_event_name(sm_event_t event)
{
    switch (event) {
    case SM_EVENT_NONE:
        return "none";
    case SM_EVENT_ARM:
        return "arm";
    case SM_EVENT_START_CALIBRATION:
        return "start_calibration";
    case SM_EVENT_CALIBRATION_DONE:
        return "calibration_done";
    case SM_EVENT_START_MAPPING:
        return "start_mapping";
    case SM_EVENT_START_RACING:
        return "start_racing";
    case SM_EVENT_ENTER_MANUAL:
        return "enter_manual";
    case SM_EVENT_STOP:
        return "stop";
    default:
        return "unknown";
    }
}
