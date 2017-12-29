#ifndef RECEIVER_H
#define RECEIVER_H

#include "settings.h"
#include "pins.h"

class channels_t {
    public:
        uint16_t channels[NUM_CHANNELS];
};

void read_receiver(channels_t *channels);
void init_rx_interrupts();

#endif // RECEIVER_H
