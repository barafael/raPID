#ifndef RECEIVER_H
#define RECEIVER_H

#include "../settings.h"
#include "../util.h"

using channels_t = int16_t[NUM_CHANNELS];

class Receiver {
    public:
        virtual const void update(channels_t channels) = 0;
        virtual const bool has_signal() = 0;
        virtual void set_trims(channels_t trims) = 0;
};

#endif // RECEIVER_H
