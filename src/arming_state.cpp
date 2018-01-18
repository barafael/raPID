#include "../include/arming_state.h"

static uint64_t disarm_init_time;
static uint64_t arm_init_time;

static inline bool channels_disarming_input(channels_t channels) {
    if (channels[THROTTLE_CHANNEL] > 25)   return false;
    if (channels[YAW_CHANNEL]      > -475) return false;
    if (channels[PITCH_CHANNEL]    > -475) return false;
    if (channels[ROLL_CHANNEL]     > -475) return false;
    return true;
}

bool disarming_input(channels_t channels) {
    return channels_disarming_input(channels);
}

void disarm_init() {
    disarm_init_time = millis();
}

bool disarming_complete() {
    uint64_t elapsed = millis() - disarm_init_time;
    return elapsed > DISARM_TIMEOUT;
}

bool arming_input(channels_t channels) {
    return channels_disarming_input(channels);
}

void arm_init() {
    arm_init_time = millis();
}

bool arming_complete() {
    uint64_t elapsed = millis() - arm_init_time;
    return elapsed > ARM_TIMEOUT;
}
