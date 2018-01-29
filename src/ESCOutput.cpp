#include "../include/ESCOutput.h"

/* TODO implement arming for ESC type? */
/* TODO implement flight mode offset? */
/* TODO implement servo/ESC subclasses? */
/* TODO better model for position/thrust/endpoints */
/* TODO remove includes when not using warning output */

static const uint16_t THROTTLE_LOW_CUTOFF = 25;
static const uint16_t BASE_PULSE_MS = 800;

ESCOutput::ESCOutput(output_type type, uint8_t pin)
    : out_type       { type }
    , pin            { pin }
    , mixer          { 0.0, 0.0, 0.0, 0.0 }
    , inverted       { false }
    , upper_limit    { 1000 }
    , lower_limit    { 0 }
    , milli_throttle { 0 } {
    output.attach(pin);

    if (upper_limit < lower_limit) {
        uint16_t tmp = lower_limit;
        lower_limit = upper_limit;
        upper_limit = tmp;
        Serial.print("Dubious limits given to output on pin ");
        Serial.println(pin);
    }
    range = upper_limit - lower_limit;
}

void ESCOutput::invert_servo_direction() {
    if(out_type == SERVO) {
        inverted = !inverted;
    }
}

bool ESCOutput::is_inverted() {
    return inverted;
}

void ESCOutput::shut_off() {
    write(0);
}

void ESCOutput::write(uint16_t _milli_throttle) {
    _milli_throttle = _milli_throttle > 1000 ? 1000 : _milli_throttle;
    uint16_t thrust = (uint32_t) (this->range * _milli_throttle) / 1000;
    if (inverted) {
        output.writeMicroseconds(upper_limit - thrust);
    } else {
        output.writeMicroseconds(BASE_PULSE_MS + lower_limit + thrust);
    }
    milli_throttle = _milli_throttle;
}

void ESCOutput::apply(uint16_t _milli_throttle,
        float roll_stbl, float pitch_stbl, float yaw_stbl) {

    /* Throttle cutoff to avoid spinning props due to movement when throttle is low but state is armed
     * Do it here, before the control values are added up
     */
    if (out_type == ESC && _milli_throttle < THROTTLE_LOW_CUTOFF) {
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

ESCOutput *ESCOutput::set_limits(uint16_t lower, uint16_t upper) {
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
    return this;
}

ESCOutput *ESCOutput::set_throttle_volume(float volume) {
    mixer.throttle_volume = volume;
    return this;
}

ESCOutput *ESCOutput::set_roll_volume(float volume) {
    mixer.roll_volume = volume;
    return this;
}

ESCOutput *ESCOutput::set_pitch_volume(float volume) {
    mixer.pitch_volume = volume;
    return this;
}

ESCOutput *ESCOutput::set_yaw_volume(float volume) {
    mixer.yaw_volume = volume;
    return this;
}
