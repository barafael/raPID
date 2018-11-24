#ifndef PWMRECEIVER_H
#define PWMRECEIVER_H

#include <stdbool.h>

#include "../ArduinoMock.h"
#include "../settings.h"
#include "../util.h"

#define PWM_RECEIVER_NOT_INITIALIZED 0
#define PWM_RECEIVER_INITIALIZED 1
//@ ghost int pwmreceiver_status = PWM_RECEIVER_NOT_INITIALIZED;

#define PWM_NUM_CHANNELS NUM_CHANNELS
typedef struct {
    /* Channel offsets and throttle zero-point */
    int16_t offsets[NUM_CHANNELS];

    /* Per-channel trims */
    int16_t trims[NUM_CHANNELS];

    /* Per-channel inversion */
    bool inversion[NUM_CHANNELS];
} PWMReceiver_t;

void PWMReceiver_init(PWMReceiver_t *self, uint8_t throttle_pin, uint8_t roll_pin, uint8_t pitch_pin, uint8_t yaw_pin,
        uint8_t aux1_pin, uint8_t aux2_pin,
        const int16_t offsets[NUM_CHANNELS]);

const void receiver_update(PWMReceiver_t *self, int16_t channels[NUM_CHANNELS]);

void set_offsets(PWMReceiver_t *self, int16_t *offsets);
void set_trims(PWMReceiver_t *self, int16_t trims[NUM_CHANNELS]);
void set_inversion(PWMReceiver_t *self, bool inversion[NUM_CHANNELS]);

const bool has_signal(PWMReceiver_t *self);

void update_throttle();
void update_roll();
void update_pitch();
void update_yaw();
void update_aux1();
void update_aux2();

#endif // PWMRECEIVER_H
