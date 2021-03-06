#include "../../include/output/ServoOutput.hpp"

void ServoOutput::write(uint16_t _value) {
    this->value    = _value > 1000 ? 1000 : _value;
    uint16_t angle = (uint32_t)(this->range * value) / 1000;
    if (inverted) {
        output.writeMicroseconds(upper_limit - angle);
    } else {
        output.writeMicroseconds(BASE_PULSE_MS + lower_limit + angle);
    }
}

void ServoOutput::apply(uint16_t _value,
        const float roll_stbl, const float pitch_stbl, const float yaw_stbl) {
    /* intermediary int16_t to prevent overflow */
    int16_t value = _value;

    value =  (int16_t) (value      * mixer.throttle_volume);

    value += (int16_t) (roll_stbl  * mixer.roll_volume);
    value += (int16_t) (pitch_stbl * mixer.pitch_volume);
    value += (int16_t) (yaw_stbl   * mixer.yaw_volume);

    clamp(value, 0, 1000);

    write(value);
}

const bool ServoOutput::is_inverted() {
    return inverted;
}

void ServoOutput::invert(bool invert) {
    inverted = invert;
}

/* TODO: move to superclass */
/* TODO: improve output limit/scaling model */
void ServoOutput::set_limits(uint16_t lower, uint16_t upper) {
    if (upper < lower) {
        Serial.print(F("Dubious limits given to output on pin "));
        Serial.println(pin);
        uint16_t tmp = lower;
        lower        = upper;
        upper        = tmp;
    }
    upper_limit = upper > 1300 ? 1300 : upper;
    lower_limit = lower;

    range = upper_limit - lower_limit;
}

void ServoOutput::set_throttle_volume(float volume) {
    mixer.throttle_volume = volume;
}

void ServoOutput::set_roll_volume(float volume) {
    mixer.roll_volume = volume;
}

void ServoOutput::set_pitch_volume(float volume) {
    mixer.pitch_volume = volume;
}

void ServoOutput::set_yaw_volume(float volume) {
    mixer.yaw_volume = volume;
}
