#include "../../include/pid/CascadedPIDController.hpp"

CascadedPIDController::CascadedPIDController(
        PIDParams& primary_params,
        PIDParams& secondary_params)
    : primary   { primary_params }
    , secondary { secondary_params } {}

void CascadedPIDController::set_mode(controller_mode mode) {
    this->mode = mode;
}

float CascadedPIDController::compute(
              const float measured_angle,
              const float measured_velocity,
              const float setpoint) {
    float primary_result = primary.compute(measured_angle, setpoint);
    return secondary.compute(measured_velocity, primary_result);
}

