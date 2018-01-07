#ifndef RECEIVER_H
#define RECEIVER_H

#include "settings.h"
#include "pins.h"

/*
typedef struct {
    uint16_t& operator[](size_t i) { return c[i]; }
    uint16_t c[NUM_CHANNELS] = { 0 };
} channels_t;
*/



void update_receiver(uint16_t channels[NUM_CHANNELS]);
bool has_signal();
void init_receiver();

#endif // RECEIVER_H
