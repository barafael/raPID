#include "../include/arming_state.h"

#undef USE_SERIAL

static arming_state_t *arming_state_instance = NULL;

/*@ requires \valid(input + (0 .. NUM_CHANNELS − 1));
    assigns \nothing;
    behavior not_triggered:
      assumes input[THROTTLE_CHANNEL] >   55 ||
              input[ROLL_CHANNEL]     <  445 ||
              input[PITCH_CHANNEL]    > -445 ||
              input[YAW_CHANNEL]      > -445;
      ensures \result == false;
    behavior triggered:
      assumes input[THROTTLE_CHANNEL] <=   55 &&
              input[ROLL_CHANNEL]     >=  445 &&
              input[PITCH_CHANNEL]    <= -445 &&
              input[YAW_CHANNEL]      <= -445;
      ensures \result == true;

    complete behaviors triggered, not_triggered;
    disjoint behaviors triggered, not_triggered;
*/
const bool state_transition_triggered(const int16_t input[NUM_CHANNELS]) {
    if (input[THROTTLE_CHANNEL] >   55) return false;
    if (input[ROLL_CHANNEL]     <  445) return false;
    /* No typo - roll is different from the other channels when not inverted */
    if (input[PITCH_CHANNEL]    > -445) return false;
    if (input[YAW_CHANNEL]      > -445) return false;
    return true;
}

//#define ARMING_DEBUG

/*@
    requires \valid(arming_state_instance); */
void enter_debug_mode() {
    arming_state_instance->internal_state = INTERNAL_DEBUG;
}

/*@
    requires \valid(arming_state_instance);
    requires \valid(arming_state_instance->channels);
    requires \valid((arming_state_instance->channels) + (0 .. NUM_CHANNELS-1));
    assigns \nothing;
*/
void update_arming_state() {
    bool triggered = state_transition_triggered(arming_state_instance->channels);
    switch (arming_state_instance->internal_state) {
        case INTERNAL_ARMED:
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
            Serial.println("Armed");
#endif
#endif
            if (triggered) {
                arming_state_instance->internal_state    = DISARMING;
                arming_state_instance->state_change_time = mock_millis();
            }
            break;

        case INTERNAL_DISARMED:
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
            Serial.println("Disarmed");
#endif
#endif
            if (triggered) {
                arming_state_instance->internal_state    = ARMING;
                arming_state_instance->state_change_time = mock_millis();
            }
            break;

        case DISARMING:
            if (triggered) {
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
                Serial.println("Disarming");
#endif
#endif
                if ((mock_millis() - arming_state_instance->state_change_time) > DISARM_TIMEOUT_MS) {
                    arming_state_instance->internal_state = DISARMING_STANDBY;
                } else {
                    break;
                }
            } else {
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
                Serial.println("Going back to armed");
#endif
#endif
                arming_state_instance->internal_state = INTERNAL_ARMED;
            }
            break;

        case DISARMING_STANDBY:
            if (triggered) {
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
                Serial.println("Release the hold to complete disarming!");
#endif
#endif
            } else {
                arming_state_instance->internal_state = INTERNAL_DISARMED;
            }
            break;

        case ARMING:
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
            Serial.println("Arming");
#endif
#endif
            if (triggered) {
                if ((mock_millis() - arming_state_instance->state_change_time) > ARM_TIMEOUT_MS) {
                    arming_state_instance->internal_state = ARMING_STANDBY;
                } else {
                    break;
                }
            } else {
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
                Serial.println("Going back to disarmed");
#endif
#endif
                arming_state_instance->internal_state = INTERNAL_DISARMED;
            }
            break;

        case ARMING_STANDBY:
            if (triggered) {
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
                Serial.println("Release the hold to complete arming!");
#endif
#endif
            } else {
                arming_state_instance->internal_state = INTERNAL_ARMED;
            }
            break;

            /* unimplemented state? */
        default:
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
            Serial.println("Unimpl. State. Disarming!");
#endif
#endif
            arming_state_instance->internal_state = INTERNAL_DISARMED;
    }
}

/*@ ensures arming_state_instance == state;
    assigns arming_state_instance;
    ensures arming_state_initialized == ARMING_ON;
*/
void init_arming_state(arming_state_t *state, int16_t channels[NUM_CHANNELS]) {
    //@ ghost arming_state_initialized = ARMING_ON;
    arming_state_instance = state;
    state->channels       = channels;
    // TODO re-enable timer; until then, call update_arming_state periodically from main loop
    /*
    if (!state->state_change_timer.begin(update_arming_state, INTERVAL_US)) {
        error_blink(STATE_TIMER_HARDWARE_BUSY, "Could not set up interval timer for arming state update!");
    }
    */
}

/*@ requires \valid(self);
    requires arming_state_initialized == ARMING_ON;
    assigns \nothing;
    ensures interrupt_status == INTERRUPTS_ON; */
const state_t get_arming_state(arming_state_t *self) {
    mock_noInterrupts();
    switch (self->internal_state) {
        case INTERNAL_DEBUG:
        case INTERNAL_ARMED:
        case DISARMING:
        case DISARMING_STANDBY: 
            mock_interrupts();
            return ARMED;
        case INTERNAL_DISARMED:
        case ARMING:
        case ARMING_STANDBY:
            mock_interrupts();
            return DISARMED;
    }
    mock_interrupts();
    return DISARMED;
}
