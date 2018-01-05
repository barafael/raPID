#include "../interface/output_mixer.h"

/* TODO remove when not using warning output in apply(x) */

#include "Arduino.h"
#include "WProgram.h"

static const uint16_t THROTTLE_LOW_CUTOFF_MS = 1025;

#define clamp(value, low, high) (value = ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value))))

Output_mixer::Output_mixer(void) {
    out_type = ESC;
    pin = 22;
    mixer_t mixer;
    mixer.throttle_vol = 100;
    mixer.volumes = { 100, 0, 0 };
    this->mixer = mixer;
    out_servo.attach(pin);
    upper_limit = 2000;
    lower_limit = 1000;
}

Output_mixer::Output_mixer(out_type_t type, uint8_t pin, mixer_t mixer) {
    out_type = type;
    this->pin = pin;
    this->mixer = mixer;
    out_servo.attach(pin);
}

void Output_mixer::apply(uint16_t throttle,
        pid_result roll_stbl, pid_result pitch_stbl, pid_result yaw_stbl
        /*,pid_result roll_rate, pid_result pitch_rate, pid_result yaw_rate*/) {

    /* Throttle cutoff to avoid spinning props due to movement when throttle is low but state is armed */
    if (out_type == ESC && throttle < THROTTLE_LOW_CUTOFF_MS) {
        out_servo.writeMicroseconds(1000);
        return;
    }

    // TODO take this out to settings init
    if (mixer.throttle_vol > 100 || mixer.throttle_vol < -100 || mixer.volumes.r_vol > 100 || mixer.volumes.r_vol < -100 || mixer.volumes.p_vol > 100 || mixer.volumes.p_vol < -100 || mixer.volumes.y_vol > 100 || mixer.volumes.y_vol < -100) {
        Serial.println("one of the volume parameters is out of range of [-100, 100]");

        clamp(mixer.throttle_vol, -100, 100);
        clamp(mixer.volumes.r_vol, -100, 100);
        clamp(mixer.volumes.p_vol, -100, 100);
        clamp(mixer.volumes.y_vol, -100, 100);
    }

    throttle *= (uint16_t) mixer.throttle_vol/100.0;

    throttle += (uint16_t) roll_stbl.sum * mixer.volumes.r_vol/100.0;

    throttle += (uint16_t) pitch_stbl.sum * mixer.volumes.p_vol/100.0;

    throttle += (uint16_t) yaw_stbl.sum * mixer.volumes.y_vol/100.0;

    /*
    throttle = throttle > upper_limit ? upper_limit : throttle;
    throttle = throttle < lower_limit ? lower_limit : throttle;
    */
    clamp(throttle, lower_limit, upper_limit);

    out_servo.writeMicroseconds(throttle);
}

