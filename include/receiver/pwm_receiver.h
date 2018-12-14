#ifndef PWM_RECEIVER_H
#define PWM_RECEIVER_H

#include <stdbool.h>

#include "../ArduinoMock.h"
#include "../settings.h"
#include "../util.h"

#define PWM_RECEIVER_NOT_INITIALIZED 0
#define PWM_RECEIVER_INITIALIZED 1

/* Interrupts write to this array and the update function reads
 * Note: disable interrupts when reading to avoid race conditions
*/
volatile int16_t channels_shared[NUM_CHANNELS];

/* Written by interrupt on rising edge, read on falling edge
 * No synchronization necessary if an interrupt only touches one array member.
 */
volatile uint64_t pwm_pulse_start_time[NUM_CHANNELS];


static int16_t pwm_offsets[NUM_CHANNELS];

/* Per-channel trims */
int16_t trims[NUM_CHANNELS];

    /* Per-channel inversion */
    bool inversion[NUM_CHANNELS];

void pwm_receiver_init(uint8_t throttle_pin, uint8_t roll_pin, uint8_t pitch_pin, uint8_t yaw_pin,
        uint8_t aux1_pin, uint8_t aux2_pin,
        const int16_t offsets[NUM_CHANNELS]);

const void receiver_update(int16_t channels[NUM_CHANNELS]);

void set_offsets(int16_t *offsets);
void set_trims(int16_t trims[NUM_CHANNELS]);
void set_inversion(bool inversion[NUM_CHANNELS]);

const bool has_signal();

void update_throttle();
void update_roll();
void update_pitch();
void update_yaw();
void update_aux1();
void update_aux2();

#endif // PWM_RECEIVER_H
