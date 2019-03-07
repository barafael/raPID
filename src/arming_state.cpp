#include "../include/arming_state.h"

#undef USE_SERIAL

//implements: GLOBAL_timestamp_type
static uint64_t state_change_time;

static internal_state_t internal_state;
static int16_t *channels;

#define TRANSITION_THROTTLE_THRESHOLD 55
#define TRANSITION_ROLL_THRESHOLD 445
#define TRANSITION_PITCH_THRESHOLD (-445)
#define TRANSITION_YAW_THRESHOLD (-445)

/*@ requires valid_access: \valid_read(input + (0 .. NUM_CHANNELS − 1));
    assigns \nothing;
    behavior not_triggered:
      assumes input[THROTTLE_CHANNEL] >  TRANSITION_THROTTLE_THRESHOLD ||
              input[ROLL_CHANNEL]     <  TRANSITION_ROLL_THRESHOLD ||
              input[PITCH_CHANNEL]    >  TRANSITION_PITCH_THRESHOLD ||
              input[YAW_CHANNEL]      >  TRANSITION_YAW_THRESHOLD;
      ensures \result == false;
    behavior triggered:
      assumes input[THROTTLE_CHANNEL] <=   TRANSITION_THROTTLE_THRESHOLD &&
              input[ROLL_CHANNEL]     >=   TRANSITION_ROLL_THRESHOLD &&
              input[PITCH_CHANNEL]    <=   TRANSITION_PITCH_THRESHOLD &&
              input[YAW_CHANNEL]      <=   TRANSITION_YAW_THRESHOLD;
      ensures \result == true;

    complete behaviors triggered, not_triggered;
    disjoint behaviors triggered, not_triggered;
*/
const bool state_transition_triggered(const int16_t input[NUM_CHANNELS]) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(input + 0); */
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(input + 1); */
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(input + 2); */
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(input + 3); */
    if (input[THROTTLE_CHANNEL] > TRANSITION_THROTTLE_THRESHOLD) return false;
    if (input[ROLL_CHANNEL]     < TRANSITION_ROLL_THRESHOLD)     return false;
    /* No typo - roll is different from the other channels when not inverted */
    if (input[PITCH_CHANNEL]    > TRANSITION_PITCH_THRESHOLD)    return false;
    if (input[YAW_CHANNEL]      > TRANSITION_YAW_THRESHOLD)      return false;
    return true;
}

/*@
    requires ARM_init: ghost_arming_init_state == ARMING_INITIALIZED;
*/
void enter_debug_mode() {
    internal_state = INTERNAL_DEBUG;
}

/*@
    requires ARM_init: ghost_arming_init_state == ARMING_INITIALIZED;
    requires \valid_read(channels);
    requires \valid_read(channels + (0 .. NUM_CHANNELS - 1));
    assigns milliseconds;
    assigns internal_state;
    assigns state_change_time;
    ensures internal_state == ARMING_STANDBY ==> \at(internal_state, Post) == ARMING_STANDBY || \at(internal_state, Post) == ARMED;
    ensures ARM_statemachine: internal_state == INTERNAL_ARMED ||
            internal_state == INTERNAL_DISARMED ||
            internal_state == ARMING ||
            internal_state == DISARMING ||
            internal_state == ARMING_STANDBY ||
            internal_state == DISARMING_STANDBY;
    ensures ARM_statemachine: \old(internal_state)  == INTERNAL_ARMED ==>
        \at(internal_state, Post) == INTERNAL_ARMED ||
        \at(internal_state, Post) == DISARMING;
    ensures ARM_statemachine: \old(internal_state)  == INTERNAL_DISARMED ==>
        \at(internal_state, Post) == INTERNAL_DISARMED ||
        \at(internal_state, Post) == ARMING;

    ensures ARM_statemachine: \old(internal_state)  == DISARMING ==>
        \at(internal_state, Post) == DISARMING ||
        \at(internal_state, Post) == INTERNAL_ARMED ||
        \at(internal_state, Post) == DISARMING_STANDBY;
    ensures ARM_statemachine: \old(internal_state)  == ARMING ==>
        \at(internal_state, Post) == ARMING ||
        \at(internal_state, Post) == INTERNAL_DISARMED ||
        \at(internal_state, Post) == ARMING_STANDBY;

    ensures ARM_statemachine: \old(internal_state)  == ARMING_STANDBY ==>
        \at(internal_state, Post) == ARMING_STANDBY ||
        \at(internal_state, Post) == INTERNAL_ARMED;
    ensures ARM_statemachine: \old(internal_state)  == DISARMING_STANDBY ==>
        \at(internal_state, Post) == DISARMING_STANDBY ||
        \at(internal_state, Post) == INTERNAL_DISARMED;
*/
void update_arming_state() {
    bool triggered = state_transition_triggered(channels);
    switch (internal_state) {
        case INTERNAL_ARMED:
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
            Serial.println("Armed");
#endif
#endif
            if (triggered) {
                internal_state    = DISARMING;
                //implements: GLOBAL_elapsed_time, GLOBAL_elapsed_time_calculation
                state_change_time = mock_millis();
            }
            break;

        case INTERNAL_DISARMED:
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
            Serial.println("Disarmed");
#endif
#endif
            if (triggered) {
                internal_state    = ARMING;
                //implements: GLOBAL_elapsed_time, GLOBAL_elapsed_time_calculation
                state_change_time = mock_millis();
            }
            break;

        case DISARMING:
            if (triggered) {
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
                Serial.println("Disarming");
#endif
#endif
                //implements: GLOBAL_elapsed_time, GLOBAL_elapsed_time_calculation
                if ((mock_millis() - state_change_time) > DISARM_TIMEOUT_MS) {
                    internal_state = DISARMING_STANDBY;
                } else {
                    break;
                }
            } else {
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
                Serial.println("Going back to armed");
#endif
#endif
                internal_state = INTERNAL_ARMED;
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
                internal_state = INTERNAL_DISARMED;
            }
            break;

        case ARMING:
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
            Serial.println("Arming");
#endif
#endif
            if (triggered) {
                //implements: GLOBAL_elapsed_time, GLOBAL_elapsed_time_calculation
                if ((mock_millis() - state_change_time) > ARM_TIMEOUT_MS) {
                    internal_state = ARMING_STANDBY;
                } else {
                    break;
                }
            } else {
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
                Serial.println("Going back to disarmed");
#endif
#endif
                internal_state = INTERNAL_DISARMED;
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
                internal_state = INTERNAL_ARMED;
            }
            break;

            /* unimplemented state? */
        default:
#ifdef ARMING_DEBUG
#ifdef USE_SERIAL
            Serial.println("Unimpl. State. Disarming!");
#endif
#endif
            internal_state = INTERNAL_DISARMED;
    }
}

/*@
    requires \valid(channels);
    requires \valid(channels + (0 .. NUM_CHANNELS - 1));

    requires ARM_init: ghost_arming_init_state == ARMING_NOT_INITIALIZED;

    assigns channels;
    assigns ghost_arming_init_state;
    ensures channels == _channels;

    ensures ARM_init: ghost_arming_init_state == ARMING_INITIALIZED;
*/
void init_arming_state(int16_t _channels[NUM_CHANNELS]) {
    channels = _channels;
    // TODO re-enable timer; until then, call update_arming_state periodically from main loop
    /*
    if (!state_change_timer.begin(update_arming_state, INTERVAL_US)) {
        error_blink(STATE_TIMER_HARDWARE_BUSY, "Could not set up interval timer for arming state update!");
    }
    */
    //implements: ARM_init
    //@ ghost ghost_arming_init_state = ARMING_INITIALIZED;
}

/*@
    requires ARM_init: ghost_arming_init_state == ARMING_INITIALIZED;
    //assigns \nothing;
    assigns ghost_interrupt_status;
    ensures ARM_armed_XOR_disarmed: \result == ARMED || \result == DISARMED;
    ensures ARM_interrupt_safety: GLOBAL_interrupt_reenable: ghost_interrupt_status == INTERRUPTS_ON;
*/
const arming_state_t get_arming_state() {
    mock_noInterrupts();
    //@ assert GLOBAL_interrupt_safety: ghost_interrupt_status == INTERRUPTS_OFF;
    // non-functional execution property: critical section is longer than it needs to be
    // (internal state is read in a switch, critical section could be shorter by using aux variable)
    switch (internal_state) {
        case INTERNAL_DEBUG:
        case INTERNAL_ARMED:
        case DISARMING:
        case DISARMING_STANDBY:
            mock_interrupts();
            //@ assert ARM_interrupt_safety: GLOBAL_interrupt_reenable: ghost_interrupt_status == INTERRUPTS_ON;
            return ARMED;
        case INTERNAL_DISARMED:
        case ARMING:
        case ARMING_STANDBY:
            mock_interrupts();
            //@ assert ARM_interrupt_safety: GLOBAL_interrupt_reenable: ghost_interrupt_status == INTERRUPTS_ON;
            return DISARMED;
    }
    mock_interrupts();
    //@ assert ARM_interrupt_safety: GLOBAL_interrupt_reenable: ghost_interrupt_status == INTERRUPTS_ON;
    return DISARMED;
}
