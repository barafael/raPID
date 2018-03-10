#ifndef CASCADED_PID_CONTROLLER_H
#define CASCADED_PID_CONTROLLER_H

#include <stdint.h>

#include "PIDController.hpp"

typedef enum { STABILIZE, RATE } controller_mode;

class CascadedPIDController {
    private:
        controller_mode mode = STABILIZE;

        PIDController primary;
        PIDController secondary;

    public:
        CascadedPIDController(PIDParams& primary_params, PIDParams& secondary_params);

        void set_mode(controller_mode mode);

        float compute(const float measured_angle,
                  const float measured_velocity,
                  const float setpoint);
};

#endif // CASCADED_PID_CONTROLLER_H

