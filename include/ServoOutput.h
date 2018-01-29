#ifndef SERVO_OUTPUT_H
#define SERVO_OUTPUT_H

#include "Arduino.h"

#include "Output.h"

#include "Servo.h"

#include "./util.h"

static const uint16_t BASE_PULSE_MS = 800;

class ServoOutput : Output {
    private:
        Servo output;

        /* TODO: make necessary or remove */
        uint16_t value = 0;

        uint16_t upper_limit = 1000;
        uint16_t lower_limit = 0;
        uint16_t range = upper_limit - lower_limit;

        bool inverted = false;

        void write(uint16_t value);

    public:
        ServoOutput(const uint8_t pin,
                float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume)
            : Output(pin, throttle_volume, roll_volume, pitch_volume, yaw_volume) {
                output.attach(pin);
            }

        void apply(uint16_t value,
                const float roll_stbl, const float pitch_stbl, const float yaw_stbl);

        bool is_inverted();
        void invert();

        void set_limits(uint16_t lower, uint16_t upper);

        void set_throttle_volume(float volume);

        void set_roll_volume (float volume);
        void set_pitch_volume(float volume);
        void set_yaw_volume  (float volume);
};

#endif // SERVO_OUTPUT_H

