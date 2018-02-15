#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <cstddef>

static const size_t NUM_CHANNELS = 6;

static const uint16_t THROTTLE_LOW_CUTOFF = 25;

typedef enum {
    THROTTLE_CHANNEL = 0,
    ROLL_CHANNEL     = 1,
    PITCH_CHANNEL    = 2,
    YAW_CHANNEL      = 3,

    AUX1_CHANNEL     = 4,
    AUX2_CHANNEL     = 5
} input_channel;

#endif // SETTINGS_H
