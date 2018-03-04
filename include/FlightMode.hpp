#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

#include "pid/PIDParams.hpp"
#include "output/Mixer.hpp"

class FlightMode {
    private:
        PIDParams<float> roll_stbl;
        PIDParams<float> roll_rate;

        PIDParams<float> pitch_stbl;
        PIDParams<float> pitch_rate;

        PIDParams<float> yaw_stbl;
        PIDParams<float> yaw_rate;

        Mixer left_mixer;
        Mixer right_mixer;
        Mixer front_mixer;
        Mixer back_mixer;

    public:
        FlightMode(PIDParams<float> roll_params_stbl, PIDParams<float> roll_params_rate,
                PIDParams<float> pitch_params_stbl,   PIDParams<float> pitch_params_rate,
                PIDParams<float> yaw_params_stbl,     PIDParams<float> yaw_params_rate,
                Mixer left_mixer,  Mixer right_mixer,
                Mixer front_mixer, Mixer back_mixer);

        void set_roll_stbl(PIDParams<float>& stbl);
        void set_roll_rate(PIDParams<float>& rate);

        void set_pitch_stbl(PIDParams<float>& stbl);
        void set_pitch_rate(PIDParams<float>& rate);

        void set_yaw_stbl(PIDParams<float>& stbl);
        void set_yaw_rate(PIDParams<float>& rate);

        void set_left_mixer(float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume);

        void set_right_mixer(float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume);

        void set_front_mixer(float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume);

        void set_back_mixer(float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume);
};

#endif // FLIGHTMODE_H

