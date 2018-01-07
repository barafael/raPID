#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>

const static uint16_t DISARM_THRESHOLD = 1150;
const static uint16_t DISARM_TIMEOUT = 2500;

const static uint16_t ARM_THRESHOLD = 1150;
const static uint16_t ARM_TIMEOUT = 2500;

const static uint16_t CONFIG_THRESHOLD = 1150;
const static uint16_t CONFIG_TIMEOUT = 1500;

static const uint8_t NUM_CHANNELS = 6;

#endif // SETTINGS_H
