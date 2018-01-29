#ifndef ESC_OUTPUT_H
#define ESC_OUTPUT_H

#include "Arduino.h"

#include <stdint.h>

#include "Servo.h"
#include "util.h"
#include "Mixer.h"

typedef enum { SERVO, ESC } output_type;

typedef enum { PWM_STANDARD, PWM_FAST } pwm_frequency;

class ESCOutput {
    private:
        output_type out_type;
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
        ESCOutput(const output_type type, const uint8_t pin);
        void shut_off();
        void apply(uint16_t throttle,
                float roll_stbl, float pitch_stbl, float yaw_stbl//,
                /* what params are needed? */
                /* float roll_rate, float pitch_rate, float yaw_rate*/);

        bool is_inverted();
        void invert_servo_direction();

        ESCOutput *set_limits(uint16_t lower, uint16_t upper);

        ESCOutput *set_throttle_volume(float volume);

        ESCOutput *set_roll_volume (float volume);
        ESCOutput *set_pitch_volume(float volume);
        ESCOutput *set_yaw_volume  (float volume);
};

#endif // ESC_OUTPUT_H

