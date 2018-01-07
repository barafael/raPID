#include "Arduino.h"
#include "WProgram.h"

#include "Servo.h"

#include "../interface/imu.h"
#include "../interface/settings.h"
#include "../interface/state.h"
#include "../interface/watchdog.h"

static uint64_t disarm_init_time;
static uint64_t arm_init_time;

static inline bool channels_within_threshold(uint16_t channels[NUM_CHANNELS], const int threshold) {
    bool in_threshold = true;
    if (channels[THROTTLE_CHANNEL] > threshold) in_threshold = false;
    if (channels[YAW_CHANNEL]      > threshold) in_threshold = false;
    if (channels[PITCH_CHANNEL]    > threshold) in_threshold = false;
    if (channels[ROLL_CHANNEL]     > threshold) in_threshold = false;
    return in_threshold;
}

bool disarming_input(uint16_t channels[NUM_CHANNELS]) {
    return channels_within_threshold(channels, DISARM_THRESHOLD);
}

void disarm_init() {
    disarm_init_time = millis();
}

bool disarming_complete() {
    uint64_t elapsed = millis() - disarm_init_time;
    return elapsed > DISARM_TIMEOUT;
}

bool arming_input(uint16_t channels[NUM_CHANNELS]) {
    return channels_within_threshold(channels, DISARM_THRESHOLD);
}

void arm_init() {
    arm_init_time = millis();
}

bool arming_complete() {
    uint64_t elapsed = millis() - arm_init_time;
    return elapsed > ARM_TIMEOUT;
}
