#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

#include "PIDParams.h"
#include "Mixer.h"

class FlightMode {
    private:
        PIDParams roll_stbl;
        PIDParams roll_rate;

        PIDParams pitch_stbl;
        PIDParams pitch_rate;

        PIDParams yaw_stbl;
        PIDParams yaw_rate;

        Mixer left_mixer;
        Mixer right_mixer;
        Mixer front_mixer;
        Mixer back_mixer;

    public:
        FlightMode(PIDParams roll_params_stbl, PIDParams roll_params_rate,
                PIDParams pitch_params_stbl,   PIDParams pitch_params_rate,
                PIDParams yaw_params_stbl,     PIDParams yaw_params_rate,
                Mixer left_mixer,  Mixer right_mixer,
                Mixer front_mixer, Mixer back_mixer);

        void set_roll_stbl(PIDParams stbl) {
            this->roll_stbl = stbl;
        }

        void set_roll_rate(PIDParams rate) {
            this->roll_rate = rate;
        }

        void set_pitch_stbl(PIDParams stbl) {
            this->pitch_stbl = stbl;
        }

        void set_pitch_rate(PIDParams rate) {
            this->pitch_rate = rate;
        }

        void set_yaw_stbl(PIDParams stbl) {
            this->yaw_stbl = stbl;
        }

        void set_yaw_rate(PIDParams rate) {
            this->yaw_rate = rate;
        }

        void set_left_mixer(float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume) {
            this->left_mixer.throttle_volume  = throttle_volume;
            this->left_mixer.roll_volume      = roll_volume;
            this->left_mixer.pitch_volume     = pitch_volume;
            this->left_mixer.yaw_volume       = yaw_volume;
        }

        void set_right_mixer(float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume) {
            this->right_mixer.throttle_volume = throttle_volume;
            this->right_mixer.roll_volume     = roll_volume;
            this->right_mixer.pitch_volume    = pitch_volume;
            this->right_mixer.yaw_volume      = yaw_volume;
        }

        void set_front_mixer(float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume) {
            this->front_mixer.throttle_volume = throttle_volume;
            this->front_mixer.roll_volume     = roll_volume;
            this->front_mixer.pitch_volume    = pitch_volume;
            this->front_mixer.yaw_volume      = yaw_volume;
        }

        void set_back_mixer(float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume) {
            this->back_mixer.throttle_volume  = throttle_volume;
            this->back_mixer.roll_volume      = roll_volume;
            this->back_mixer.pitch_volume     = pitch_volume;
            this->back_mixer.yaw_volume       = yaw_volume;
        }
};

#endif // FLIGHTMODE_H

