#ifndef STATE_H
#define STATE_H

// TRANSITION
// PASS_THROUGH
typedef enum {
    ARMED, DISARMED, CONFIG
} state_t;

void arm();

void disarm();

void config();

#endif // STATE_H
