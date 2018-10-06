#include <stdbool.h>

#include "../../include/output/FastPWMOutput.h"

FastPWMOutput_t fast_out_init(const uint8_t pin,
        float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume,
        bool is_throttle) {
    uint8_t resolution_bits = 16;
    float frequency_hz = 400.0f;
    float wavelength_sec = 1.0f/frequency_hz;

    float min_pulse_width_sec = 0.001f;
    float max_pulse_width_sec = 0.002f;

    uint8_t min_dutycycle_percent = (min_pulse_width_sec/wavelength_sec) * 100.0f;
    uint8_t max_dutycycle_percent = (max_pulse_width_sec/wavelength_sec) * 100.0f;

    uint16_t lower_limit = (1u << resolution_bits) * (min_dutycycle_percent / 100.0f);
    uint16_t upper_limit = (1u << resolution_bits) * (max_dutycycle_percent / 100.0f);

    uint16_t range = upper_limit - lower_limit;

    analogWriteResolution(resolution_bits);
    pinMode(pin, OUTPUT);
    analogWriteFrequency(pin, frequency_hz);

    mixer_t mixer = mixer_init(throttle_volume, roll_volume, pitch_volume, yaw_volume);

    FastPWMOutput_t result = {
        .pin = pin,
        .mixer = mixer,
        .milli_throttle = 0,
        .lower_limit = lower_limit,
        .upper_limit = upper_limit,
        .range = range,
        .low_throttle_cutoff_enabled = is_throttle,
        .throttle_low_cutoff = 25
    };
    return result;
}

void fast_out_write(FastPWMOutput_t *self, uint16_t _milli_throttle) {
    self->milli_throttle = _milli_throttle > 1000 ? 1000 : _milli_throttle;
    // reusing _millithrottle could result in error if clamp was necessary
    float thrust = self->milli_throttle / 1000.0;
    analogWrite(self->pin, self->lower_limit + self->range * thrust);
}

void fast_out_apply(FastPWMOutput_t *self, uint16_t _milli_throttle,
        float roll_stbl, float pitch_stbl, float yaw_stbl) {
    /* Throttle cutoff to avoid spinning props due to movement when throttle is low but state is armed
     * Do it here, before the control values are added up */
    if (self->low_throttle_cutoff_enabled && (_milli_throttle < self->throttle_low_cutoff)) {
        fast_out_shutoff(self);
        return;
    }

    /* intermediary int16_t to prevent overflow */
    int16_t throttle_tmp = (int16_t) (_milli_throttle * self->mixer.throttle_volume);

    throttle_tmp += (int16_t) (roll_stbl  * self->mixer.roll_volume);
    throttle_tmp += (int16_t) (pitch_stbl * self->mixer.pitch_volume);
    throttle_tmp += (int16_t) (yaw_stbl   * self->mixer.yaw_volume);

    clamp(throttle_tmp, 0, 1000);

    fast_out_write(self, throttle_tmp);
}

void fast_out_shutoff(FastPWMOutput_t *self) {
    fast_out_write(self, 0);
}

void fast_out_set_limits(FastPWMOutput_t* self, uint16_t lower, uint16_t upper) {
    if (upper < lower) {
        //Serial.print(F("Dubious limits given to output on pin "));
        //Serial.println(self->pin);
        uint16_t tmp = lower;
        lower        = upper;
        upper        = tmp;
    }
    self->upper_limit = upper > 1300 ? 1300 : upper;
    self->lower_limit = lower;

    // do not reuse unclamped values
    self->range = self->upper_limit - self->lower_limit;
}

void fast_out_set_throttle_volume(FastPWMOutput_t *self, float volume) {
    self->mixer.throttle_volume = volume;
}

void fast_out_set_roll_volume    (FastPWMOutput_t *self, float volume) {
    self->mixer.roll_volume = volume;
}

void fast_out_set_pitch_volume   (FastPWMOutput_t *self, float volume) {
    self->mixer.pitch_volume = volume;
}

void fast_out_set_yaw_volume     (FastPWMOutput_t *self, float volume) {
    self->mixer.yaw_volume = volume;
}
