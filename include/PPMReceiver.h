#ifndef PPMRECEIVER_H
#define PPMRECEIVER_H

#include "PulsePosition.h"

#include "settings.h"
#include "Receiver.h"

class PPMReceiver : Receiver {
    private:
        uint8_t input_pin;
        PulsePositionInput input_ppm;
        int count = 0;

    public:
        PPMReceiver(uint8_t _input_pin);
        
        const void update(channels_t channels);
        const bool has_signal();
};

#endif // PPMRECEIVER_H
