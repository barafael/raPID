#include "../include/PPMReceiver.h"

#define clamp(value, low, high) \
    ((value) = \
    ((value) < (low)  ? (low) : \
    ((value) > (high) ? (high) : (value))))

PPMReceiver::PPMReceiver(uint8_t _input_pin) {
    input_pin = _input_pin;
    input_ppm.begin(input_pin);
}

const void PPMReceiver::update(channels_t channels) {
    int num = input_ppm.available();
    if (num > 0) {
        count = count + 1;
        for (size_t index = 0; index < NUM_CHANNELS; index++) {
            float val = input_ppm.read(index + 1);
            channels[index] = (uint16_t) val;
            clamp(channels[index], 1000, 2000);
            channels[index] -= 1500;
        }
        channels[THROTTLE_CHANNEL] += 500;
    }
}

const bool PPMReceiver::has_signal() {
    return true;
}
