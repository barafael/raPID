#include "../include/ArmingState.hpp"

ArmingState *arming_state_instance = nullptr;

static const bool state_transition_triggered(channels_t input) {
    if (input[THROTTLE_CHANNEL] >   55) return false;
    if (input[ROLL_CHANNEL]     <  445) return false;
    /* No typo - roll is different from the other channels when not inverted */
    if (input[PITCH_CHANNEL]    > -445) return false;
    if (input[YAW_CHANNEL]      > -445) return false;
    return true;
}

static const bool arming_debug = true;

// #define ARMING_DEBUG

void ArmingState::enter_debug_mode() {
    arming_state_instance->internal_state = INTERNAL_DEBUG;
}

void update_state() {
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
                if ((millis() - arming_state_instance->state_change_time) > arming_state_instance->DISARM_TIMEOUT_MS) {
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
                if ((millis() - arming_state_instance->state_change_time) > arming_state_instance->ARM_TIMEOUT_MS) {
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

ArmingState::ArmingState(channels_t channels) : channels(channels) {
    arming_state_instance = this;
    if (!state_change_timer.begin(update_state, INTERVAL_US)) {
        error_blink(STATE_TIMER_HARDWARE_BUSY, "Could not set up interval timer for arming state update!");
    }
}

const state_t ArmingState::get_state() {
    noInterrupts();
    switch (internal_state) {
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
