#include "../include/FastPWMOutput.h"

/* TODO implement arming for ESC type? */
/* TODO implement flight mode offset? */
/* TODO implement servo/ESC subclasses? */
/* TODO better model for position/thrust/endpoints */
/* TODO remove includes when not using warning output */
 
void FastPWMOutput::shut_off() {
    write(0);
}

/* Avoiding invert functionality, maybe simplify? */
void FastPWMOutput::write(uint16_t _milli_throttle) {
    this->milli_throttle = _milli_throttle > 1000 ? 1000 : _milli_throttle;
    uint16_t thrust = (uint32_t) (this->range * milli_throttle) / 1000;
    float value = thrust / 1000.0f;
    analogWrite(pin, lower_limit + range * value);
}

void FastPWMOutput::apply(uint16_t _milli_throttle,
        float roll_stbl, float pitch_stbl, float yaw_stbl) {
    /* Throttle cutoff to avoid spinning props due to movement when throttle is low but state is armed
     * Do it here, before the control values are added up
     */
    if (low_throttle_cutoff_enabled && (_milli_throttle < THROTTLE_LOW_CUTOFF)) {
        shut_off();
        return;
    }

    /* intermediary int16_t to prevent overflow */
    int16_t throttle_tmp = _milli_throttle;

    throttle_tmp =  (int16_t) (_milli_throttle * mixer.throttle_volume);

    throttle_tmp += (int16_t) (roll_stbl       * mixer.roll_volume);
    throttle_tmp += (int16_t) (pitch_stbl      * mixer.pitch_volume);
    throttle_tmp += (int16_t) (yaw_stbl        * mixer.yaw_volume);

    clamp(throttle_tmp, 0, 1000);

    write(throttle_tmp);
}

void FastPWMOutput::set_resolution(uint8_t res) {
    if (res <= 16) {
        resolution_bits = res;
    }
}

void FastPWMOutput::set_frequency(uint32_t freq) {
        frequency_hz = freq;
}

void FastPWMOutput::set_limits(uint16_t lower, uint16_t upper) {
    if (upper < lower) {
        Serial.print("Dubious limits given to output on pin ");
        Serial.println(pin);
        uint16_t tmp = lower;
        lower = upper;
        upper = tmp;
    }
    upper_limit = upper > 1300 ? 1300 : upper;
    lower_limit = lower;

    range = upper_limit - lower_limit;
}

void FastPWMOutput::set_throttle_cutoff_enabled(bool enable) {
    low_throttle_cutoff_enabled = enable;
}

void FastPWMOutput::set_throttle_volume(float volume) {
    mixer.throttle_volume = volume;
}

void FastPWMOutput::set_roll_volume(float volume) {
    mixer.roll_volume = volume;
}

void FastPWMOutput::set_pitch_volume(float volume) {
    mixer.pitch_volume = volume;
}

void FastPWMOutput::set_yaw_volume(float volume) {
    mixer.yaw_volume = volume;
}
