#include "../../include/output/LEDOutput.hpp"

/* TODO implement arming for ESC type? */
/* TODO better model for position/thrust/endpoints */
/* TODO remove includes when not using warning output */

LEDOutput::LEDOutput(const uint8_t pin,
        float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume)
    : Output(pin, throttle_volume, roll_volume, pitch_volume, yaw_volume) {
        analogWriteResolution(resolution_bits);
    pinMode(pin, OUTPUT);
    analogWriteFrequency(pin, frequency_hz);
}

void LEDOutput::shut_off() {
    write(0);
}

/* Avoiding invert functionality, maybe simplify? */
void LEDOutput::write(uint16_t _brightness) {
    this->brightness = _brightness > 1000 ? 1000 : _brightness;
    float thrust = brightness / 1000.0;
    analogWrite(pin, lower_limit + range * thrust);
}

void LEDOutput::apply(uint16_t _brightness,
        float roll_stbl, float pitch_stbl, float yaw_stbl) {
    /* Throttle cutoff to avoid spinning props due to movement when throttle is low but state is armed
     * Do it here, before the control values are added up
     */
    if (low_throttle_cutoff_enabled && (_brightness < THROTTLE_LOW_CUTOFF)) {
        shut_off();
        return;
    }

    /* intermediary int16_t to prevent overflow */
    int16_t throttle_tmp = (int16_t) (_brightness * mixer.throttle_volume);

    throttle_tmp += (int16_t) (roll_stbl  * mixer.roll_volume);
    throttle_tmp += (int16_t) (pitch_stbl * mixer.pitch_volume);
    throttle_tmp += (int16_t) (yaw_stbl   * mixer.yaw_volume);

    clamp(throttle_tmp, 0, 1000);

    write(throttle_tmp);
}

void LEDOutput::set_resolution(uint8_t res) {
    if (res <= 16) {
        resolution_bits = res;
    }
}

void LEDOutput::set_frequency(uint32_t freq) {
        frequency_hz = freq;
}

void LEDOutput::set_limits(uint16_t lower, uint16_t upper) {
    if (upper < lower) {
        Serial.print(F("Dubious limits given to output on pin "));
        Serial.println(pin);
        uint16_t tmp = lower;
        lower = upper;
        upper = tmp;
    }
    upper_limit = upper > 1300 ? 1300 : upper;
    lower_limit = lower;

    range = upper_limit - lower_limit;
}

void LEDOutput::set_throttle_cutoff_enabled(bool enable) {
    low_throttle_cutoff_enabled = enable;
}

void LEDOutput::set_throttle_volume(float volume) {
    mixer.throttle_volume = volume;
}

void LEDOutput::set_roll_volume(float volume) {
    mixer.roll_volume = volume;
}

void LEDOutput::set_pitch_volume(float volume) {
    mixer.pitch_volume = volume;
}

void LEDOutput::set_yaw_volume(float volume) {
    mixer.yaw_volume = volume;
}
