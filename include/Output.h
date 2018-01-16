#ifndef OUTPUT_H
#define OUTPUT_H

#include <stdint.h>
#include "Servo.h"

#include "../include/PIDController.h"

class Mixer {
    private:
    public:
        float throttle_volume;

        float roll_volume;
        float pitch_volume;
        float yaw_volume;
};

typedef enum { SERVO, ESC, STEPPER } out_type_t;
// typedef enum { STBL, RATE } mode_t;

class Output {
    public:
        Output();
        Output(const out_type_t type, const uint8_t pin);
        void shut_off();
        void apply(uint16_t throttle,
                float roll_stbl, float pitch_stbl, float yaw_stbl//,
                /* what params are needed? */
                /* float roll_rate, float pitch_rate, float yaw_rate*/);

        Output* set_throttle_volume(float volume);

        Output* set_roll_volume  (float volume);
        Output* set_pitch_volume (float volume);
        Output* set_yaw_volume   (float volume);

    private:
        // mode_t mode;
        out_type_t out_type;
        uint8_t pin;
        Mixer mixer;
        uint16_t upper_limit = 1000;
        uint16_t lower_limit = 0;
        //uint16_t throttle = 0;
        Servo output;
};

#endif // OUTPUT_H

