#include "../../include/FlightMode.hpp"

#include "../../include/pid/PIDParams.hpp"
#include "../../include/output/Mixer.hpp"

/* TODO implement flight mode offset? */

void FlightMode::set_roll_stbl(PIDParams<float> *stbl) {
    this->roll_stbl = *stbl;
}

void FlightMode::set_roll_rate(PIDParams<float> *rate) {
    this->roll_rate = *rate;
}

void FlightMode::set_pitch_stbl(PIDParams<float> *stbl) {
    this->pitch_stbl = *stbl;
}

void FlightMode::set_pitch_rate(PIDParams<float> *rate) {
    this->pitch_rate = *rate;
}

void FlightMode::set_yaw_stbl(PIDParams<float> *stbl) {
    this->yaw_stbl = *stbl;
}

void FlightMode::set_yaw_rate(PIDParams<float> *rate) {
    this->yaw_rate = *rate;
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

