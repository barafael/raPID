#ifndef PPMRECEIVER_H
#define PPMRECEIVER_H

#include<stdbool.h>

#include "PulsePosition.h"
#include "../settings.h"
#include "../util.h"

typedef struct {
    uint8_t input_pin;
    PulsePositionInput ppm_rx;

    int16_t ppm_translate[NUM_CHANNELS] = { 2, 0, 1, 3, 4, 5 };

    /* Channel offsets and throttle zero-point */
    int16_t offsets[NUM_CHANNELS];

    /* Per-channel trims */
    int16_t trims[NUM_CHANNELS];

    /* Per-channel inversion */
    bool inversion[NUM_CHANNELS];
} PPMReceiver_t;

PPMReceiver_t init_ppm_receiver(uint8_t _input_pin, int16_t *offsets);

const void receiver_update(int16_t *channels);

void set_offsets(int16_t *offsets);
void set_trims(int16_t *trims);
void set_inversion(bool *inversion);

const bool has_signal();

#endif // PPMRECEIVER_H
