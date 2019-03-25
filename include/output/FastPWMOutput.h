#ifndef MISRAC_FAST_PWM_OUTPUT
#define MISRAC_FAST_PWM_OUTPUT

#include "../Mock.h"

#include "../util.h"

#include "mixer.h"

typedef struct FastPWMOutput_t {
    uint8_t pin;

    mixer_t mixer;

    uint16_t milli_throttle;

    uint16_t lower_limit;
    uint16_t upper_limit;

    uint16_t range;

    bool low_throttle_cutoff_enabled;
    uint16_t throttle_low_cutoff;

} FastPWMOutput_t;

FastPWMOutput_t fast_out_init(uint8_t pin,
        float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume,
        bool is_throttle);

void fast_out_write(FastPWMOutput_t *self, uint16_t _milli_throttle);

void fast_out_apply(FastPWMOutput_t *self, uint16_t _milli_throttle,
        float roll_stbl, float pitch_stbl, float yaw_stbl);

void fast_out_shutoff(FastPWMOutput_t *self);

void fast_out_set_limits(FastPWMOutput_t *self, uint16_t lower, uint16_t upper);

void fast_out_set_throttle_volume(FastPWMOutput_t *self, float volume);
void fast_out_set_roll_volume    (FastPWMOutput_t *self, float volume);
void fast_out_set_pitch_volume   (FastPWMOutput_t *self, float volume);
void fast_out_set_yaw_volume     (FastPWMOutput_t *self, float volume);

#endif // MISRAC_FAST_PWM_OUTPUT
