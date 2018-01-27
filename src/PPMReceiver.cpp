#include "../include/PPMReceiver.h"

PPMReceiver::PPMReceiver(uint8_t _input_pin) {
    input_pin = _input_pin;
    ppm_rx.begin(input_pin);
}

const void PPMReceiver::update(channels_t channels) {
    int num = ppm_rx.available();
    if (num > 0) {
        count = count + 1;
        for (size_t index = 0; index < NUM_CHANNELS; index++) {
            float val = ppm_rx.read(index + 1);
            channels[index] = (uint16_t) val;
            clamp(channels[index], 1000, 2000);
            channels[index] -= 1500;
        }
        channels[THROTTLE_CHANNEL] += 500;
    }
}

/* TODO less pointless function */
const bool PPMReceiver::has_signal() {
    return true;
}
