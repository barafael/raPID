#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

#include "../../include/output/mixer.h"
#include "../../include/pid/PIDParams.h"

class FlightMode {
    private:
    pid_params_t roll_stbl;
    pid_params_t roll_rate;

    pid_params_t pitch_stbl;
    pid_params_t pitch_rate;

    pid_params_t yaw_stbl;
    pid_params_t yaw_rate;

    mixer_t left_mixer;
    mixer_t right_mixer;
    mixer_t front_mixer;
    mixer_t back_mixer;

    public:
    FlightMode(pid_params_t roll_params_stbl, pid_params_t roll_params_rate, pid_params_t pitch_params_stbl,
               pid_params_t pitch_params_rate, pid_params_t yaw_params_stbl, pid_params_t yaw_params_rate,
               mixer_t left_mixer, mixer_t right_mixer, mixer_t front_mixer, mixer_t back_mixer);

    void set_roll_stbl(pid_params_t *stbl);
    void set_roll_rate(pid_params_t *rate);

    void set_pitch_stbl(pid_params_t *stbl);
    void set_pitch_rate(pid_params_t *rate);

    void set_yaw_stbl(pid_params_t *stbl);
    void set_yaw_rate(pid_params_t *rate);

    void set_left_mixer(float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume);

    void set_right_mixer(float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume);

    void set_front_mixer(float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume);

    void set_back_mixer(float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume);
};

#endif // FLIGHTMODE_H
