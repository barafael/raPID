#ifndef PWMRECEIVER_H
#define PWMRECEIVER_H

#include <vector>
#include "../settings.h"
#include "../util.h"

using channels_t = int16_t[NUM_CHANNELS];
using inversion_t = bool[NUM_CHANNELS];

typedef struct {
    channels_t pins = { 0 };

    /* Interrupts write to this array and the update function reads
     * Note: disable interrupts when reading to avoid race conditions
     */
    volatile channels_t channels_shared = { 0 };

    /* Written by interrupt on rising edge, read on falling edge
     * No synchronization necessary if an interrupt only touches one array member.
     */
    volatile channels_t pwm_pulse_start_time = { 0 };

    /* Channel offsets and throttle zero-point */
    channels_t offsets = { 0 };

    /* Per-channel trims */
    channels_t trims = { 0 };

    /* Per-channel inversion */
    inversion_t inversion = { false };
} PWMReceiver_t;

PWMReceiver_t PWMReceiver_init(uint8_t throttle_pin, uint8_t roll_pin, uint8_t pitch_pin, uint8_t yaw_pin,
        uint8_t aux1_pin, uint8_t aux2_pin,
        channels_t offsets);

const void update(PWMReceiver_t *self, channels_t channels);

void set_offsets(PWMReceiver_t *self, channels_t offsets);
void set_trims(PWMReceiver_t *self, channels_t trims);
void set_inversion(PWMReceiver_t *self, inversion_t inversion);

const bool has_signal(PWMReceiver_t *self);

void update_throttle();
void update_roll();
void update_pitch();
void update_yaw();
void update_aux1();
void update_aux2();

#endif // PWMRECEIVER_H
