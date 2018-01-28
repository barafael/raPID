#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>

static const uint16_t DISARM_TIMEOUT = 2500;

static const uint16_t ARM_TIMEOUT = 1000;

static const uint16_t CONFIG_TIMEOUT = 2500;

static const size_t NUM_CHANNELS = 6;

typedef enum {
    THROTTLE_CHANNEL = 0,
    ROLL_CHANNEL     = 1,
    PITCH_CHANNEL    = 2,
    YAW_CHANNEL      = 3,

    AUX1_CHANNEL     = 4,
    AUX2_CHANNEL     = 5
} input_channel;

#endif // SETTINGS_H
