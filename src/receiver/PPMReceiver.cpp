#include "../../include/receiver/PPMReceiver.hpp"

PPMReceiver_t init_ppm_receiver(uint8_t input_pin, int16_t *offsets) {
    PPMReceiver_t receiver;
    switch (input_pin) {
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
            Serial.print(input_pin);
            Serial.println(" cannot be used as pulse position input.");
    }

    receiver.input_pin = input_pin;

    /* TODO use init list? */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        receiver.offsets[index] = offsets[index];
    }
    receiver.ppm_rx.begin(input_pin);
    return receiver;
}

const void receiver_update(PPMReceiver_t *self, int16_t *channels) {
    int num = self->ppm_rx.available();
    if (num > 0) {
        for (size_t index = 0; index < NUM_CHANNELS; index++) {
            float val = self->ppm_rx.read(index + 1);
            clamp(val, 1000, 2000);
            if (self->inversion[index]) {
                val = 2000 - (val - 1000);
            }
            val += self->offsets[index];
            val += self->trims[index];
            channels[self->ppm_translate[index]] = (int16_t) val;
        }
    }
}

void set_trims(PPMReceiver_t *self, int16_t *trims) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->trims[index] = trims[index];
    }
}

void set_inversion(PPMReceiver_t *self, bool *inversion) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->inversion[index] = inversion[index];
    }
}

/* TODO less pointless function */
const bool has_signal() {
    return true;
}
