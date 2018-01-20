#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

#include "PIDParams.h"
#include "Mixer.h"

class FlightMode {
    private:
        PIDParams roll_params_stbl;
        PIDParams roll_params_rate;

        PIDParams pitch_params_stbl;
        PIDParams pitch_params_rate;

        PIDParams yaw_params_rate;

        Mixer mixer_left;

        Mixer mixer_right;

        Mixer mixer_front;

        Mixer mixer_back;
};

#endif //FLIGHTMODE_H

