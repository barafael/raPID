#ifndef OUTPUT_H
#define OUTPUT_H

#include <stdint.h>
#include "Servo.h"

#include "../interface/pid_controller.h"

class rpy_volume_t {
    public:
        int8_t r_vol;
        int8_t p_vol;
        int8_t y_vol;
};

class mixer_t {
    public:
        int8_t throttle_vol;

        rpy_volume_t volumes;
};

typedef enum { SERVO, ESC } out_type_t;
// typedef enum { STBL, RATE } mode_t;

class Output{
    public:
        Output();
        Output(out_type_t type, uint8_t pin, mixer_t mixer);
        void shut_off();
        void apply(uint16_t throttle,
                float roll_stbl, float pitch_stbl, float yaw_stbl//,
                /* float roll_rate, float pitch_rate, float yaw_rate*/);

    private:
        mixer_t mixer;
        uint8_t pin;
        // mode_t mode;
        out_type_t out_type;
        uint16_t upper_limit = 2000;
        uint16_t lower_limit = 1000;
        uint16_t throttle = 0;
        Servo output;
};

#endif // OUTPUT_H
