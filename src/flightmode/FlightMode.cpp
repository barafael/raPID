#include "../../include/FlightMode.h"

/* TODO implement flight mode offset? */

void FlightMode::set_roll_stbl(pid_params_t *stbl) {
    this->roll_stbl.p_gain = stbl->p_gain;
    this->roll_stbl.i_gain = stbl->i_gain;
    this->roll_stbl.d_gain = stbl->d_gain;
}

void FlightMode::set_roll_rate(pid_params_t *rate) {
    this->roll_rate.p_gain = rate->p_gain;
    this->roll_rate.i_gain = rate->i_gain;
    this->roll_rate.d_gain = rate->d_gain;
}

void FlightMode::set_pitch_stbl(pid_params_t *stbl) {
    this->pitch_stbl.p_gain = stbl->p_gain;
    this->pitch_stbl.i_gain = stbl->i_gain;
    this->pitch_stbl.d_gain = stbl->d_gain;
}

void FlightMode::set_pitch_rate(pid_params_t *rate) {
    this->pitch_rate.p_gain = rate->p_gain;
    this->pitch_rate.i_gain = rate->i_gain;
    this->pitch_rate.d_gain = rate->d_gain;
}

void FlightMode::set_yaw_stbl(pid_params_t *stbl) {
    this->yaw_stbl.p_gain = stbl->p_gain;
    this->yaw_stbl.i_gain = stbl->i_gain;
    this->yaw_stbl.d_gain = stbl->d_gain;
}

void FlightMode::set_yaw_rate(pid_params_t *rate) {
    this->yaw_rate.p_gain = rate->p_gain;
    this->yaw_rate.i_gain = rate->i_gain;
    this->yaw_rate.d_gain = rate->d_gain;
}

void FlightMode::set_left_mixer(float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume) {
    this->left_mixer.throttle_volume  = throttle_volume;
    this->left_mixer.roll_volume      = roll_volume;
    this->left_mixer.pitch_volume     = pitch_volume;
    this->left_mixer.yaw_volume       = yaw_volume;
}

void FlightMode::set_right_mixer(float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume) {
    this->right_mixer.throttle_volume = throttle_volume;
    this->right_mixer.roll_volume     = roll_volume;
    this->right_mixer.pitch_volume    = pitch_volume;
    this->right_mixer.yaw_volume      = yaw_volume;
}

void FlightMode::set_front_mixer(float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume) {
    this->front_mixer.throttle_volume = throttle_volume;
    this->front_mixer.roll_volume     = roll_volume;
    this->front_mixer.pitch_volume    = pitch_volume;
    this->front_mixer.yaw_volume      = yaw_volume;
}

void FlightMode::set_back_mixer(float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume) {
    this->back_mixer.throttle_volume  = throttle_volume;
    this->back_mixer.roll_volume      = roll_volume;
    this->back_mixer.pitch_volume     = pitch_volume;
    this->back_mixer.yaw_volume       = yaw_volume;
}

