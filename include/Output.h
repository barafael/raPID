#ifndef OUTPUT_H
#define OUTPUT_H

#include "Arduino.h"

#include <stdint.h>

#include "Servo.h"

class Mixer {
    private:
    public:
        float throttle_volume;

        float roll_volume;
        float pitch_volume;
        float yaw_volume;
};

typedef enum { SERVO, ESC, STEPPER } out_type_t;

class Output {
    private:
        out_type_t out_type;
        uint8_t pin;
        Mixer mixer;
        bool inverted = false;
        uint16_t upper_limit = 1000;
        uint16_t lower_limit = 0;
        Servo output;
        uint16_t milli_throttle = 0;
        uint16_t range = upper_limit - lower_limit;

        void write(uint16_t _milli_throttle);

    public:
        Output(const out_type_t type, const uint8_t pin);
        void shut_off();
        void apply(uint16_t throttle,
                float roll_stbl, float pitch_stbl, float yaw_stbl//,
                /* what params are needed? */
                /* float roll_rate, float pitch_rate, float yaw_rate*/);

        bool is_inverted();
        void invert_servo_direction();

        Output *set_limits(uint16_t lower, uint16_t upper);

        Output *set_throttle_volume(float volume);

        Output *set_roll_volume    (float volume);
        Output *set_pitch_volume   (float volume);
        Output *set_yaw_volume     (float volume);
};

#endif // OUTPUT_H

