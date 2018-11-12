#include "../../include/flightmode.h"

/* TODO implement flight mode offset? */

void flightmode_set_roll_stbl(flightmode_t *self, pid_param_t stbl) {
    self->roll_stbl = stbl;
}

void flightmode_set_roll_rate(flightmode_t *self, pid_param_t rate) {
    self->roll_rate = rate;
}

void flightmode_set_pitch_stbl(flightmode_t *self, pid_param_t stbl) {
    self->pitch_stbl = stbl;
}

void flightmode_set_pitch_rate(flightmode_t *self, pid_param_t rate) {
    self->pitch_rate = rate;
}

void flightmode_set_yaw_stbl(flightmode_t *self, pid_param_t stbl) {
    self->yaw_stbl = stbl;
}

void flightmode_set_yaw_rate(flightmode_t *self, pid_param_t rate) {
    self->yaw_rate = rate;
}

void flightmode_set_left_mixer(flightmode_t *self, float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume) {
    self->left_mixer.throttle_volume  = throttle_volume;
    self->left_mixer.roll_volume      = roll_volume;
    self->left_mixer.pitch_volume     = pitch_volume;
    self->left_mixer.yaw_volume       = yaw_volume;
}

void flightmode_set_right_mixer(flightmode_t *self, float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume) {
    self->right_mixer.throttle_volume = throttle_volume;
    self->right_mixer.roll_volume     = roll_volume;
    self->right_mixer.pitch_volume    = pitch_volume;
    self->right_mixer.yaw_volume      = yaw_volume;
}

void flightmode_set_front_mixer(flightmode_t *self, float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume) {
    self->front_mixer.throttle_volume = throttle_volume;
    self->front_mixer.roll_volume     = roll_volume;
    self->front_mixer.pitch_volume    = pitch_volume;
    self->front_mixer.yaw_volume      = yaw_volume;
}

void flightmode_set_back_mixer(flightmode_t *self, float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume) {
    self->back_mixer.throttle_volume  = throttle_volume;
    self->back_mixer.roll_volume      = roll_volume;
    self->back_mixer.pitch_volume     = pitch_volume;
    self->back_mixer.yaw_volume       = yaw_volume;
}

