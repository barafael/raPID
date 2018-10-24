#ifndef MISRAC_SIMPLE_PWM_OUTPUT
#define MISRAC_SIMPLE_PWM_OUTPUT

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

} SimplePWMOutput_t;

SimplePWMOutput_t simple_out_init(uint8_t pin,
        float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume,
        bool is_throttle);

void simple_out_write(SimplePWMOutput_t *self, uint16_t _milli_throttle);

void simple_out_apply(SimplePWMOutput_t *self, uint16_t _milli_throttle,
        float roll_stbl, float pitch_stbl, float yaw_stbl);

void simple_out_shutoff(SimplePWMOutput_t *self);

void simple_out_set_limits(SimplePWMOutput_t *self, uint16_t lower, uint16_t upper);

void simple_out_set_throttle_volume(SimplePWMOutput_t *self, float volume);
void simple_out_set_roll_volume    (SimplePWMOutput_t *self, float volume);
void simple_out_set_pitch_volume   (SimplePWMOutput_t *self, float volume);
void simple_out_set_yaw_volume     (SimplePWMOutput_t *self, float volume);

#endif // MISRAC_SIMPLE_PWM_OUTPUT
