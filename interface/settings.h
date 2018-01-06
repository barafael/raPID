#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>

const static uint16_t DISARM_THRESHOLD = 1150;
const static uint16_t DISARM_TIMEOUT = 2500;

const static uint16_t ARM_THRESHOLD = 1150;
const static uint16_t ARM_TIMEOUT = 2500;

const static uint16_t CONFIG_THRESHOLD = 1150;
const static uint16_t CONFIG_TIMEOUT = 1500;

typedef enum {
    ROLL_RATE  = 0,
    PITCH_RATE = 1,
    YAW_RATE   = 2
} angle_rate;

static const uint8_t NUM_CHANNELS = 6;

typedef enum {
    THROTTLE_CHANNEL  = 0,
    ROLL_CHANNEL      = 1,
    PITCH_CHANNEL     = 2,
    YAW_CHANNEL       = 3,
    AUX1_CHANNEL      = 4,
    AUX2_CHANNEL      = 5
} input_channel;

#endif // SETTINGS_H
