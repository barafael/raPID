#ifndef RECEIVER_H
#define RECEIVER_H

#include "settings.h"
#include "pins.h"

typedef struct {
    uint16_t channels[NUM_CHANNELS];
} channels_t;

void read_receiver(channels_t *channels);
void init_rx_interrupts();

#endif // RECEIVER_H
