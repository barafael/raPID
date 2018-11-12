#ifndef SIMPLE_PWM_OUTPUT
#define SIMPLE_PWM_OUTPUT

#include <Arduino.h>

#include "../util.h"

#include "mixer.h"

typedef struct {
    uint8_t pin;

    mixer_t mixer;

    uint16_t milli_throttle;

    uint16_t lower_limit;
    uint16_t upper_limit;

    uint16_t range;

    bool low_throttle_cutoff_enabled;
    uint16_t throttle_low_cutoff;

} simple_pwm_output_t;

simple_pwm_output_t simple_out_init(uint8_t pin,
        float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume,
        bool is_throttle);

void simple_out_write(simple_pwm_output_t *self, uint16_t _milli_throttle);

void simple_out_apply(simple_pwm_output_t *self, uint16_t _milli_throttle,
        float roll_stbl, float pitch_stbl, float yaw_stbl);

void simple_out_shutoff(simple_pwm_output_t *self);

void simple_out_set_limits(simple_pwm_output_t *self, uint16_t lower, uint16_t upper);

void simple_out_set_throttle_volume(simple_pwm_output_t *self, float volume);
void simple_out_set_roll_volume    (simple_pwm_output_t *self, float volume);
void simple_out_set_pitch_volume   (simple_pwm_output_t *self, float volume);
void simple_out_set_yaw_volume     (simple_pwm_output_t *self, float volume);

#endif // SIMPLE_PWM_OUTPUT
