#include "Arduino.h"
#include "WProgram.h"

#include "../interface/output.h"

/* TODO support inverting output */
/* TODO support arming for ESC type? */
/* TODO support lower and upper limit everywhere */
/* TODO remove includes  when not using warning output in apply(x) */
static const uint16_t THROTTLE_LOW_CUTOFF_MS = 1025;

#define clamp(value, low, high) ((value) = ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value))))

Output::Output()
    : mixer { 100, { 100, 0, 0 }}
    , pin { 22 }
    , out_type { ESC }
    , upper_limit { 1000 }
    , lower_limit { 1000 }
{
    output.attach(pin);
}

Output::Output(out_type_t type, uint8_t pin, mixer_t mixer) 
    : mixer { mixer }
    , pin { pin }
    , out_type { type }
    , upper_limit { 1000 }
    , lower_limit { 1000 }
{
    output.attach(pin);
}

void Output::shut_off() {
    output.writeMicroseconds(lower_limit);
}

void Output::apply(uint16_t throttle, float roll_stbl, float pitch_stbl, float yaw_stbl
                         /*,float roll_rate, float pitch_rate, float yaw_rate*/) {

    /* Throttle cutoff to avoid spinning props due to movement when throttle is low but state is armed */
    if (out_type == ESC && throttle < THROTTLE_LOW_CUTOFF_MS) {
        output.writeMicroseconds(lower_limit);
        return;
    }

    // TODO take this out to settings init
    if (mixer.throttle_vol  > 100  || mixer.throttle_vol  < -100 || mixer.volumes.r_vol > 100 ||
        mixer.volumes.r_vol < -100 || mixer.volumes.p_vol > 100  || mixer.volumes.p_vol < -100 ||
        mixer.volumes.y_vol > 100  || mixer.volumes.y_vol < -100) {

        Serial.println("one of the volume parameters is out of range of [-100, 100]");

        clamp(mixer.throttle_vol,  -100, 100);
        clamp(mixer.volumes.r_vol, -100, 100);
        clamp(mixer.volumes.p_vol, -100, 100);
        clamp(mixer.volumes.y_vol, -100, 100);
    }

    throttle = 1000 + (throttle - 1000) * (uint16_t) mixer.throttle_vol / 100.0;

    throttle += (uint16_t) roll_stbl *  (mixer.volumes.r_vol / 100.0);

    throttle += (uint16_t) pitch_stbl * (mixer.volumes.p_vol / 100.0);

    throttle += (uint16_t) yaw_stbl *   (mixer.volumes.y_vol / 100.0);

    clamp(throttle, lower_limit, upper_limit);

    output.writeMicroseconds(throttle);
}
