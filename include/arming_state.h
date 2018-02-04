#ifndef ARMING_STATE_H
#define ARMING_STATE_H

#include "Arduino.h"

#include "settings.h"
#include "Receiver.h"

// TRANSITION
// PASS_THROUGH
// FAILSAFE
typedef enum {
    DISARMED,
    ARMING,
    ARMED,
    DISARMING,
    CONFIG
} state_t;

void disarm_init();
void arm_init();

bool disarming_input(channels_t channels);
bool arming_input(channels_t channels);

bool disarming_complete();
bool arming_complete();

#endif // ARMING_STATE_H
