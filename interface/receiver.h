#ifndef RECEIVER_H
#define RECEIVER_H

#include "settings.h"
#include "pins.h"

class channels_t {
    public:
        uint16_t channels[NUM_CHANNELS];
};

void update_receiver(channels_t *channels);
bool init_receiver();

#endif // RECEIVER_H
