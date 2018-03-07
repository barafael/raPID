#ifndef PPMRECEIVER_H
#define PPMRECEIVER_H

#include "PulsePosition.h"

#include "settings.h"
#include "Receiver.hpp"

class PPMReceiver : Receiver {
    private:
        uint8_t input_pin;
        PulsePositionInput ppm_rx;

        channels_t ppm_translate = { 2, 0, 1, 3, 4, 5 };

    public:
        PPMReceiver(uint8_t _input_pin, channels_t offsets);

        const void update(channels_t channels) override;

        void set_offsets(channels_t channels) override;
        void set_trims(channels_t channels) override;
        void set_inversion(inversion_t inversion) override;

        const bool has_signal() override;
};

#endif // PPMRECEIVER_H
