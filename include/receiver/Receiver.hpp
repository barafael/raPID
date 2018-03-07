#ifndef RECEIVER_H
#define RECEIVER_H

#include "../settings.h"
#include "../util.h"

using channels_t = int16_t[NUM_CHANNELS];
using inversion_t = bool[NUM_CHANNELS];

class Receiver {
    public:
        virtual const void update(channels_t channels) = 0;
        virtual const bool has_signal() = 0;

        virtual void set_offsets(channels_t offsets) = 0;
        virtual void set_trims(channels_t trims) = 0;
        virtual void set_inversion(inversion_t inversions) = 0;

    protected:
        /* Channel offsets and throttle zero-point */
        channels_t offsets = { 0 };

        /* Per-channel trims */
        channels_t trims = { 0 };

        /* Per-channel inversion */
        inversion_t inversion = { false };
};

#endif // RECEIVER_H
