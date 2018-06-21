#include "../../include/pid/PIDController.hpp"

PIDController::PIDController(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit)
    : p_gain(p_gain)
    , i_gain(i_gain)
    , d_gain(d_gain)

    , integral_limit (integral_limit)
    , output_limit   (output_limit) {}

PIDController::PIDController(PIDParams& params)
    : p_gain(params.p_gain)
    , i_gain(params.i_gain)
    , d_gain(params.d_gain)

    , integral_limit (params.integral_limit)
    , output_limit   (params.output_limit) {}

void PIDController::set_enabled(bool enable) {
    enabled = enable;
}

float PIDController::compute(const float measured, const float setpoint) {
    if (!enabled) {
        return setpoint;
    }

    uint64_t now = micros();

    /* If there is overflow, the elapsed time is still correct
     * The calculation overflows just like the timer
     */
    uint64_t elapsed_time = now - last_time;

    float error = measured - setpoint;

    /* Give me some P! */
    float p_term = p_gain * error;

    /* Give me some I! */
    integral += elapsed_time * error * i_gain;
    /* Integral windup clamp */
    clamp(integral, -integral_limit, integral_limit);

    /* Give me some D! */
    float d_term = 0.0f;
    switch (d_type) {
        /* Derivative term on error */
        case ERROR:
            d_term = ((error - last_error) / elapsed_time) * d_gain;
            break;

        /* Derivative term on setpoint */
        case SETPOINT:
            d_term = ((setpoint - last_setpoint) / elapsed_time) * d_gain;
            break;

        /* Derivative term on measurement */
        case FEEDBACK:
            d_term = ((measured - last_measured) / elapsed_time) * d_gain;
            break;
    }

    if (deriv_filter_enabled && deriv_filter != nullptr) {
        d_term = deriv_filter->next(d_term);
    }

    last_time     = now;

    last_error    = error;
    last_setpoint = setpoint;
    last_measured = measured;

    float result = p_term + integral + d_term;

    /* Output limit */
    clamp(result, -output_limit, output_limit);

    return result;
}

void PIDController::set_derivative_type(derivative_type type) {
    this->d_type = type;
}

void PIDController::set_p(const float _p_gain) {
    this->p_gain = _p_gain;
}

void PIDController::set_i(const float _i_gain) {
    this->i_gain = _i_gain;
}

void PIDController::set_d(const float _d_gain) {
    this->d_gain = _d_gain;
}

const float PIDController::get_p() {
    return this->p_gain;
}

const float PIDController::get_i() {
    return this->i_gain;
}

const float PIDController::get_d() {
    return this->d_gain;
}

void PIDController::set_params(const PIDParams& params) {
    p_gain = params.p_gain;
    i_gain = params.i_gain;
    d_gain = params.d_gain;
}

void PIDController::integral_reset() {
    integral = 0.0f;
}

void PIDController::set_filter(Filter *filter) {
    this->deriv_filter = filter;
    this->deriv_filter_enabled = true;
}

void PIDController::set_enable_derivative_filter(bool enable) {
    deriv_filter_enabled = enable;
}
