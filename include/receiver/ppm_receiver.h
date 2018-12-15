#ifndef PPMRECEIVER_H
#define PPMRECEIVER_H

#include <stdbool.h>

#include "PulsePosition.h"

#include "./../settings.h"
#include "./../util.h"

static uint8_t ppm_input_pin;
static PulsePositionInput ppm_rx;

static int16_t ppm_translate[NUM_CHANNELS] = { 2, 0, 1, 3, 4, 5 };

/* Channel offsets and throttle zero-point */
static int16_t ppm_offsets[NUM_CHANNELS];

/* Per-channel trims */
static int16_t ppm_trims[NUM_CHANNELS];

/* Per-channel inversion */
static bool ppm_inversion[NUM_CHANNELS];

void init_ppm_receiver(uint8_t _input_pin, int16_t offsets[NUM_CHANNELS]);

const void ppm_receiver_update(int16_t channels[NUM_CHANNELS]);

void ppm_set_offsets(int16_t offsets[NUM_CHANNELS]);
void ppm_set_trims(int16_t trims[NUM_CHANNELS]);
void ppm_set_inversion(bool inversion[NUM_CHANNELS]);

const bool ppm_has_signal();

#endif // PPMRECEIVER_H
