#ifndef PPMRECEIVER_H
#define PPMRECEIVER_H

#include "PulsePosition.h"

#include "settings.h"
#include "receiver/Receiver.h"

class PPMReceiver : Receiver {
    private:
        uint8_t input_pin;
        PulsePositionInput ppm_rx;

        /* Channel offsets and throttle zero-point */
        channels_t offsets = { 0 };

        /* Per-channel trim */
        channels_t trims = { 0 };

    public:
        PPMReceiver(uint8_t _input_pin, channels_t offsets);

        const void update(channels_t channels) override;

        void set_trims(channels_t channels) override;

        const bool has_signal() override;
};

#endif // PPMRECEIVER_H
