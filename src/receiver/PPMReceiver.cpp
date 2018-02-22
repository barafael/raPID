#include "../../include/receiver/PPMReceiver.hpp"

PPMReceiver::PPMReceiver(uint8_t _input_pin, channels_t offsets) {
    input_pin = _input_pin;

    /* TODO use init list? */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        this->offsets[index] = offsets[index];
    }
    ppm_rx.begin(input_pin);
}

const void PPMReceiver::update(channels_t channels) {
    int num = ppm_rx.available();
    if (num > 0) {
        for (size_t index = 0; index < NUM_CHANNELS; index++) {
            float val       = ppm_rx.read(index + 1);
            channels[index] = (int16_t) val;
            clamp(val, 1000, 2000);
            val += offsets[index];
        }
    }
}

void PPMReceiver::set_trims(channels_t trims) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        this->trims[index] = trims[index];
    }
}

/* TODO less pointless function */
const bool PPMReceiver::has_signal() {
    return true;
}
