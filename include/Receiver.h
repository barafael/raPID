#ifndef RECEIVER_H
#define RECEIVER_H

#include "settings.h"

using channels_t = int16_t[NUM_CHANNELS];

class Receiver {
    public:
        virtual const void update(channels_t channels);
        virtual const bool has_signal();
};

#endif // RECEIVER_H
