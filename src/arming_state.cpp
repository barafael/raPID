#include "../include/arming_state.h"

static uint64_t disarm_init_time;
static uint64_t arm_init_time;

static inline bool state_transition_triggered(channels_t input) {
    if (input[THROTTLE_CHANNEL] >   25) return false;
    if (input[ROLL_CHANNEL]     <  475) return false;
    /* No typo - roll is different from the other channels when not inverted */
    if (input[PITCH_CHANNEL]    > -475) return false;
    if (input[YAW_CHANNEL]      > -475) return false;
    return true;
}

bool disarming_input(channels_t channels) {
    return state_transition_triggered(channels);
}

void disarm_init() {
    disarm_init_time = millis();
}

bool disarming_complete() {
    uint64_t elapsed = millis() - disarm_init_time;
    return elapsed > DISARM_TIMEOUT;
}

bool arming_input(channels_t channels) {
    return state_transition_triggered(channels);
}

void arm_init() {
    arm_init_time = millis();
}

bool arming_complete() {
    uint64_t elapsed = millis() - arm_init_time;
    return elapsed > ARM_TIMEOUT;
}
