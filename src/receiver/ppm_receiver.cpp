#include "../../include/receiver/ppm_receiver.h"

void init_ppm_receiver(uint8_t _input_pin, const int16_t offsets[NUM_CHANNELS]) {
    switch (ppm_input_pin) {
        /* Filter pins that are ppm-capable */
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

    ppm_input_pin = _input_pin;

    /* TODO use init list? */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        ppm_offsets[index] = offsets[index];
    }
    ppm_rx.begin(ppm_input_pin);
}

const void ppm_receiver_update(int16_t channels[NUM_CHANNELS]) {
    int num = ppm_rx.available();
    if (num > 0) {
        for (size_t index = 0; index < NUM_CHANNELS; index++) {
            float val = ppm_rx.read(index + 1);
            clamp(val, 1000, 2000);
            if (ppm_inversion[index]) {
                val = 2000 - (val - 1000);
            }
            val += ppm_offsets[index];
            val += ppm_trims[index];
            channels[ppm_translate[index]] = (int16_t) val;
        }
    }
}

void set_trims(int16_t trims[NUM_CHANNELS]) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        ppm_trims[index] = trims[index];
    }
}

void set_inversion(bool inversion[NUM_CHANNELS]) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        ppm_inversion[index] = inversion[index];
    }
}

/* TODO less pointless function */
const bool has_signal() {
    return true;
}
