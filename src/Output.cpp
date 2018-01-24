#include "../include/Output.h"

/* TODO implement arming for ESC type? */
/* TODO implement flight mode offset? */
/* TODO remove includes when not using warning output */

static const uint16_t THROTTLE_LOW_CUTOFF = 25;
static const uint16_t BASE_PULSE_MS = 800;

#define clamp(value, low, high) \
    ((value) = \
    ((value) < (low)  ? (low) : \
    ((value) > (high) ? (high) : (value))))

Output::Output(output_type type, uint8_t pin)
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
    }
    range = upper_limit - lower_limit;
}

void Output::invert_servo_direction() {
    if(out_type == SERVO) {
        inverted = !inverted;
    }
}

bool Output::is_inverted() {
    return inverted;
}

void Output::shut_off() {
    write(0);
}

void Output::write(uint16_t _milli_throttle) {
    _milli_throttle = _milli_throttle > 1000 ? 1000 : _milli_throttle;
    uint16_t thrust = (uint32_t) (this->range * _milli_throttle) / 1000;
    if (inverted) {
        output.writeMicroseconds(upper_limit - thrust);
    } else {
        //Serial.println(BASE_PULSE_MS + lower_limit + thrust);
        output.writeMicroseconds(BASE_PULSE_MS + lower_limit + thrust);
    }
    milli_throttle = _milli_throttle;
}

void Output::apply(uint16_t _milli_throttle,
        float roll_stbl, float pitch_stbl, float yaw_stbl
        /*,float roll_rate, float pitch_rate, float yaw_rate*/) {

    /* intermediary int16_t to prevent overflow */
    int16_t throttle_tmp = _milli_throttle;
    /* Throttle cutoff to avoid spinning props due to movement when throttle is low but state is armed */
    if (out_type == ESC && _milli_throttle < THROTTLE_LOW_CUTOFF) {
        write(0);
        return;
    }

    // TODO take this out to settings init
    if (mixer.throttle_volume > 1.0 || mixer.throttle_volume < -1.0 ||
        mixer.roll_volume     > 1.0 || mixer.roll_volume     < -1.0 ||
        mixer.pitch_volume    > 1.0 || mixer.pitch_volume    < -1.0 ||
        mixer.yaw_volume      > 1.0 || mixer.yaw_volume      < -1.0) {

        Serial.println("one of the volume parameters is out of range of [-1.0, 1.0]");

        clamp(mixer.throttle_volume, -1.0, 1.0);
        clamp(mixer.roll_volume,     -1.0, 1.0);
        clamp(mixer.pitch_volume,    -1.0, 1.0);
        clamp(mixer.yaw_volume,      -1.0, 1.0);
    }

    throttle_tmp =  (int16_t) (_milli_throttle * mixer.throttle_volume);

    throttle_tmp += (int16_t) (roll_stbl       * mixer.roll_volume);

    throttle_tmp += (int16_t) (pitch_stbl      * mixer.pitch_volume);

    throttle_tmp += (int16_t) (yaw_stbl        * mixer.yaw_volume);

    clamp(throttle_tmp, 0, 1000);

    write(throttle_tmp);
}

Output *Output::set_limits(uint16_t lower, uint16_t upper) {
    if (upper > 1300 || lower > upper) {
        Serial.print("Ignoring dubious limits given to output on pin");
        Serial.println(pin);
        return this;
    }
    lower_limit = lower;
    upper_limit = upper;
    return this;
}

Output *Output::set_throttle_volume(float volume) {
    mixer.throttle_volume = volume;
    return this;
}

Output *Output::set_roll_volume(float volume) {
    mixer.roll_volume = volume;
    return this;
}

Output *Output::set_pitch_volume(float volume) {
    mixer.pitch_volume = volume;
    return this;
}

Output *Output::set_yaw_volume(float volume) {
    mixer.yaw_volume = volume;
    return this;
}
