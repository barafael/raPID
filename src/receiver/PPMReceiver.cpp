#include "../../include/receiver/PPMReceiver.hpp"

PPMReceiver::PPMReceiver(uint8_t _input_pin, channels_t offsets) {
    switch (_input_pin) {
        case 5:
        case 6:
        case 9:
        case 10:
        case 20:
        case 21:
        case 22:
        case 23:
            break;
        default:
            Serial.print("Pin ");
            Serial.print(_input_pin);
            Serial.println(" cannot be used as pulse position input.");
    }

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
            float val = ppm_rx.read(index + 1);
            clamp(val, 1000, 2000);
            val += offsets[index];
            channels[ppm_translate[index]] = (int16_t) val;
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
