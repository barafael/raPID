#include "../include/state.h"

extern state_t state;

void arm() {
    state = ARMED;
}

void disarm() {
    state = DISARMED;
}

void config() {
    state = CONFIG;
}
