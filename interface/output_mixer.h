#ifndef OUTPUT_MIXER_H
#define OUTPUT_MIXER_H

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
//typedef enum { STBL, RATE } mode_t;

class Output_mixer {
    public:
        Output_mixer();
        Output_mixer(out_type_t type, uint8_t pin, mixer_t mixer);
        void apply(uint16_t throttle,
                pid_result roll_stbl, pid_result pitch_stbl, pid_result yaw_stbl//,
                /*pid_result roll_rate, pid_result pitch_rate, pid_result yaw_rate*/);

    private:
        mixer_t mixer;
        uint8_t pin;
        //mode_t mode;
        out_type_t out_type;
        uint16_t upper_limit = 2000;
        uint16_t lower_limit = 1000;
        uint16_t throttle = 0;
        Servo out_servo;
};

#endif // OUTPUT_MIXER_H
