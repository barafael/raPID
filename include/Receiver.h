#ifndef RECEIVER_H
#define RECEIVER_H

#include "settings.h"

class Receiver {
    public:
        using channels_t = uint16_t[NUM_CHANNELS];
        virtual const void update(uint16_t channels[NUM_CHANNELS]);
        virtual const bool has_signal();
};

#endif // RECEIVER_H
