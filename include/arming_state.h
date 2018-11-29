#ifndef ARMING_STATE_H
#define ARMING_STATE_H

#include <stdbool.h>

#include "../include/settings.h"

#include "ArduinoMock.h"

#define ARMING_OFF 0
#define ARMING_ON 1

//@ ghost int ghost_arming_state_initialized = ARMING_OFF;

const bool state_transition_triggered(const int16_t input[NUM_CHANNELS]);

// TRANSITION, PASS_THROUGH, FAILSAFE, CONFIG
typedef enum { DISARMED, ARMED, DEBUG } state_t;
typedef enum {
    INTERNAL_DISARMED,
    ARMING,
    ARMING_STANDBY,
    INTERNAL_ARMED,
    DISARMING,
    DISARMING_STANDBY,
    INTERNAL_DEBUG
} internal_state_t;

typedef struct {
    internal_state_t internal_state;
    int16_t *channels;
    uint64_t state_change_time;
} arming_state_t;

/*@ requires \valid(state);
    requires \valid(channels);
    ensures state->channels == channels;
    assigns *state;
    assigns *state->channels;
    assigns ghost_arming_state_initialized;
*/
void init_arming_state(arming_state_t *state, int16_t channels[NUM_CHANNELS]);

const uint16_t DISARM_TIMEOUT_MS = 2500;
const uint16_t ARM_TIMEOUT_MS    = 1000;
/* find appropriate polling interval */
const uint32_t INTERVAL_US = 500000;

void update_arming_state();
void enter_debug_mode();

const state_t get_arming_state(arming_state_t *self);

#endif // ARMING_STATE_H
