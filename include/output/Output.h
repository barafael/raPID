#ifndef OUTPUT_H
#define OUTPUT_H

#include "Arduino.h"

#include <stdint.h>

#include "Mixer.h"

class Output {
    protected:
        uint8_t pin;
        Mixer mixer;

    public:
        explicit Output(const uint8_t pin);
        Output(const uint8_t pin,
                float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume);

        /* Pure virtual methods meant to be overridden in servo, esc, and anyPWM */
        virtual void apply(uint16_t value,
                const float roll_stbl, const float pitch_stbl, const float yaw_stbl) = 0;

        virtual void set_throttle_volume(float volume) = 0;
        virtual void set_roll_volume    (float volume) = 0;
        virtual void set_pitch_volume   (float volume) = 0;
        virtual void set_yaw_volume     (float volume) = 0;
};

#endif // OUTPUT_H

