#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

#include "output/mixer.h"
#include "pid/pid_param.h"

typedef struct {
    pid_param_t roll_stbl;
    pid_param_t roll_rate;

    pid_param_t pitch_stbl;
    pid_param_t pitch_rate;

    pid_param_t yaw_stbl;
    pid_param_t yaw_rate;

    mixer_t left_mixer;
    mixer_t right_mixer;
    mixer_t front_mixer;
    mixer_t back_mixer;
} flightmode_t;

flightmode_t init_flightmode(
        pid_param_t roll_params_stbl,  pid_param_t roll_params_rate,
        pid_param_t pitch_params_stbl, pid_param_t pitch_params_rate,
        pid_param_t yaw_params_stbl,   pid_param_t yaw_params_rate,
        mixer_t left_mixer, mixer_t right_mixer,
        mixer_t front_mixer, mixer_t back_mixer);

    void flightmode_set_roll_stbl(flightmode_t *self, pid_param_t *stbl);
    void flightmode_set_roll_rate(flightmode_t *self, pid_param_t *rate);

    void flightmode_set_pitch_stbl(flightmode_t *self, pid_param_t *stbl);
    void flightmode_set_pitch_rate(flightmode_t *self, pid_param_t *rate);

    void flightmode_set_yaw_stbl(flightmode_t *self, pid_param_t *stbl);
    void flightmode_set_yaw_rate(flightmode_t *self, pid_param_t *rate);

    void flightmode_set_left_mixer (flightmode_t *self, float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume);

    void flightmode_set_right_mixer(flightmode_t *self, float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume);

    void flightmode_set_front_mixer(flightmode_t *self, float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume);

    void flightmode_set_back_mixer (flightmode_t *self, float throttle_volume, float roll_volume, float pitch_volume, float yaw_volume);

#endif // FLIGHTMODE_H
