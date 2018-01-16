#ifndef STATE_H
#define STATE_H

using channels_t = uint16_t[NUM_CHANNELS];

// TRANSITION
// PASS_THROUGH
typedef enum {
    ARMED, DISARMING, DISARMED, ARMING, CONFIG
} state_t;

void disarm_init();
void arm_init();

bool disarming_input(channels_t channels);
bool arming_input(channels_t channels);

bool disarming_complete();
bool arming_complete();

#endif // STATE_H
