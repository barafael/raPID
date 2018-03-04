#ifndef CASCADED_PID_CONTROLLER_H
#define CASCADED_PID_CONTROLLER_H

#include <stdint.h>

#include "PIDController.hpp"

typedef enum { STABILIZE, RATE } controller_mode;

template<typename T>
class CascadedPIDController {
    private:
        controller_mode mode = STABILIZE;

        PIDController<T> primary;
        PIDController<T> secondary;

    public:
        CascadedPIDController(PIDParams<T>& primary_params, PIDParams<T>& secondary_params);

        void set_mode(controller_mode mode);

        T compute(const T measured_angle,
                  const T measured_velocity,
                  const T setpoint);
};

#include "../src/pid/CascadedPIDController.tpp"

#endif // CASCADED_PID_CONTROLLER_H

