#ifndef ARMING_STATE_H
#define ARMING_STATE_H

#include <stdbool.h>

#include "../include/settings.h"

#include "ArduinoMock.h"

#define ARMING_INITIALIZED 0
#define ARMING_OFF 1

//@ ghost int ghost_arming_state_initialized = ARMING_OFF;

const bool state_transition_triggered(const int16_t input[NUM_CHANNELS]);

// TRANSITION, PASS_THROUGH, FAILSAFE, CONFIG
typedef enum { DISARMED, ARMED, DEBUG } arming_state_t;
typedef enum {
    INTERNAL_DISARMED,
    ARMING,
    ARMING_STANDBY,
    INTERNAL_ARMED,
    DISARMING,
    DISARMING_STANDBY,
    INTERNAL_DEBUG
} internal_state_t;

void init_arming_state(int16_t _channels[NUM_CHANNELS]);

const uint16_t DISARM_TIMEOUT_MS = 2500;
const uint16_t ARM_TIMEOUT_MS    = 1000;
/* find appropriate polling interval */
const uint32_t INTERVAL_US = 500000;

void update_arming_state();
void enter_debug_mode();

const arming_state_t get_arming_state();

#endif // ARMING_STATE_H
