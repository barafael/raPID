#include "../teensy3/Arduino.h"
#include "../teensy3/WProgram.h"

#include "Servo.h"

#include "../interface/state.h"
#include "../interface/watchdog.h"
#include "../interface/imu.h"
#include "../interface/settings.h"

static uint64_t disarm_init_time;
static uint64_t arm_init_time;

static bool channels_within_threshold(channels_t *rx_input, const int threshold) {
    return rx_input->channels[THROTTLE_CHANNEL] < threshold &&
        rx_input->channels[YAW_CHANNEL] < threshold &&
        rx_input->channels[PITCH_CHANNEL] < threshold &&
        rx_input->channels[ROLL_CHANNEL] < threshold;
}

bool disarming_input(channels_t *channels) {
    return channels_within_threshold(channels, DISARM_THRESHOLD);
}

void disarm_init() {
    disarm_init_time = millis();
}

bool disarming_complete() {
    uint64_t elapsed = millis() - disarm_init_time;
    return elapsed > DISARM_TIMEOUT;
}

bool arming_input(channels_t *channels) {
    return channels_within_threshold(channels, DISARM_THRESHOLD);
}

void arm_init() {
    arm_init_time = millis();
}

bool arming_complete() {
    uint64_t elapsed = millis() - arm_init_time;
    return elapsed > ARM_TIMEOUT;
}

