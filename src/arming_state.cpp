#include "../include/arming_state.h"

arming_state_t *arming_state_instance = NULL;

const bool state_transition_triggered(int16_t *input) {
    if (input[THROTTLE_CHANNEL] >   55) return false;
    if (input[ROLL_CHANNEL]     <  445) return false;
    /* No typo - roll is different from the other channels when not inverted */
    if (input[PITCH_CHANNEL]    > -445) return false;
    if (input[YAW_CHANNEL]      > -445) return false;
    return true;
}

//#define ARMING_DEBUG

void enter_debug_mode() {
    arming_state_instance->internal_state = INTERNAL_DEBUG;
}

/* Call periodically! */
void update_arming_state() {
    bool triggered = state_transition_triggered(arming_state_instance->channels);
    switch (arming_state_instance->internal_state) {
        case INTERNAL_ARMED:
#ifdef ARMING_DEBUG
            Serial.println("Armed");
#endif
            if (triggered) {
                arming_state_instance->internal_state    = DISARMING;
                arming_state_instance->state_change_time = millis();
            }
            break;

        case INTERNAL_DISARMED:
#ifdef ARMING_DEBUG
            Serial.println("Disarmed");
#endif
            if (triggered) {
                arming_state_instance->internal_state    = ARMING;
                arming_state_instance->state_change_time = millis();
            }
            break;

        case DISARMING:
            if (triggered) {
#ifdef ARMING_DEBUG
                Serial.println("Disarming");
#endif
                if ((millis() - arming_state_instance->state_change_time) > DISARM_TIMEOUT_MS) {
                    arming_state_instance->internal_state = DISARMING_STANDBY;
                } else {
                    break;
                }
            } else {
#ifdef ARMING_DEBUG
                Serial.println("Going back to armed");
#endif
                arming_state_instance->internal_state = INTERNAL_ARMED;
            }
            break;

        case DISARMING_STANDBY:
            if (triggered) {
#ifdef ARMING_DEBUG
                Serial.println("Release the hold to complete disarming!");
#endif
            } else {
                arming_state_instance->internal_state = INTERNAL_DISARMED;
            }
            break;

        case ARMING:
#ifdef ARMING_DEBUG
            Serial.println("Arming");
#endif
            if (triggered) {
                if ((millis() - arming_state_instance->state_change_time) > ARM_TIMEOUT_MS) {
                    arming_state_instance->internal_state = ARMING_STANDBY;
                } else {
                    break;
                }
            } else {
#ifdef ARMING_DEBUG
                Serial.println("Going back to disarmed");
#endif
                arming_state_instance->internal_state = INTERNAL_DISARMED;
            }
            break;

        case ARMING_STANDBY:
            if (triggered) {
#ifdef ARMING_DEBUG
                Serial.println("Release the hold to complete arming!");
#endif
            } else {
                arming_state_instance->internal_state = INTERNAL_ARMED;
            }
            break;

            /* unimplemented state? */
        default:
#ifdef ARMING_DEBUG
            Serial.println("Unimpl. State. Disarming!");
#endif
            arming_state_instance->internal_state = INTERNAL_DISARMED;
    }
}

void init_arming_state(arming_state_t *state, int16_t *channels) {
    arming_state_instance = state;
    state->channels       = channels;
    // TODO re-enable timer; until then, use main loop
    /*
    if (!state->state_change_timer.begin(update_arming_state, INTERVAL_US)) {
        error_blink(STATE_TIMER_HARDWARE_BUSY, "Could not set up interval timer for arming state update!");
    }
    */
}

/* Prove: Interrupts always enabled after this function exits */
const state_t get_arming_state(arming_state_t *self) {
    noInterrupts();
    switch (self->internal_state) {
        case INTERNAL_DEBUG:
        case INTERNAL_ARMED:
        case DISARMING:
        case DISARMING_STANDBY: interrupts(); return ARMED;
        case INTERNAL_DISARMED:
        case ARMING:
        case ARMING_STANDBY: interrupts(); return DISARMED;
    }
    interrupts();
    return DISARMED;
}
