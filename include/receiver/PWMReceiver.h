#ifndef PWMRECEIVER_H
#define PWMRECEIVER_H

#include<stdbool.h>

#include "../settings.h"
#include "../util.h"

typedef struct {
    int16_t pins[NUM_CHANNELS];

    /* Interrupts write to this array and the update function reads
     * Note: disable interrupts when reading to avoid race conditions
     */
    volatile int16_t channels_shared[NUM_CHANNELS];

    /* Written by interrupt on rising edge, read on falling edge
     * No synchronization necessary if an interrupt only touches one array member.
     */
    volatile uint64_t pwm_pulse_start_time[NUM_CHANNELS];

    /* Channel offsets and throttle zero-point */
    int16_t offsets[NUM_CHANNELS];

    /* Per-channel trims */
    int16_t trims[NUM_CHANNELS];

    /* Per-channel inversion */
    bool inversion[NUM_CHANNELS];
} PWMReceiver_t;

void PWMReceiver_init(PWMReceiver_t *self, uint8_t throttle_pin, uint8_t roll_pin, uint8_t pitch_pin, uint8_t yaw_pin,
        uint8_t aux1_pin, uint8_t aux2_pin,
        int16_t *offsets);

const void update(PWMReceiver_t *self, int16_t *channels);

void set_offsets(PWMReceiver_t *self, int16_t *offsets);
void set_trims(PWMReceiver_t *self, int16_t *trims);
void set_inversion(PWMReceiver_t *self, bool *inversion);

bool has_signal(PWMReceiver_t *self);

void update_throttle();
void update_roll();
void update_pitch();
void update_yaw();
void update_aux1();
void update_aux2();

#endif // PWMRECEIVER_H
