#include "../../include/pid/PIDController.hpp"

PIDController::PIDController(PIDParams& params)
    : enabled ( true )

    , p_gain ( params.p_gain )
    , i_gain ( params.i_gain )
    , d_gain ( params.d_gain )

    , integral       ( 0 )
    , integral_limit ( params.integral_limit )

    , derivative     ( 0 )
    , last_error     ( 0 )
    , last_setpoint  ( 0 )
    , last_measured  ( 0 )

    , output_limit   ( params.output_limit )

    , last_time      ( 0 ) {}

PIDController::PIDController(PIDParams& params, float lowpass_beta)
    : enabled ( true )

    , p_gain ( params.p_gain )
    , i_gain ( params.i_gain )
    , d_gain ( params.d_gain )

    , integral       ( 0 )
    , integral_limit ( params.integral_limit )

    , derivative     ( 0 )
    , last_error     ( 0 )
    , last_setpoint  ( 0 )
    , last_measured  ( 0 )

    , output_limit   ( params.output_limit )

    , last_time      ( 0 ) {
        this->lowpass_beta = lowpass_beta;
        lowpass_filter = Lowpass(lowpass_beta);
        deriv_filter = &lowpass_filter;
        derivative_filter_type = LOWPASS;
}

PIDController::PIDController(PIDParams& params, size_t mov_avg_size)
    : enabled ( true )

    , p_gain ( params.p_gain )
    , i_gain ( params.i_gain )
    , d_gain ( params.d_gain )

    , integral       ( 0 )
    , integral_limit ( params.integral_limit )

    , derivative     ( 0 )
    , last_error     ( 0 )
    , last_setpoint  ( 0 )
    , last_measured  ( 0 )

    , output_limit   ( params.output_limit )

    , last_time      ( 0 ) {
        this->mov_avg_size = mov_avg_size;
        ma_filter = MovingAverage(mov_avg_size);
        deriv_filter = &ma_filter;
        derivative_filter_type = MOVING_AVERAGE;
}

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
    float p_term = this->p_gain * error;

    /* Give me some I! */
    this->integral += elapsed_time * error * this->i_gain;
    /* Integral windup clamp */
    clamp(integral, -integral_limit, integral_limit);

    /* Give me some D! */
    float d_term = 0;
    switch (d_type) {
        /* Derivative term on error */
        case ERROR:
            d_term = ((error - last_error) / elapsed_time) * d_gain;
            //Serial.println(d_term);
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

    if (deriv_filter != nullptr) {
        d_term = deriv_filter->next(d_term);
    }

    this->last_time     = now;

    this->last_error    = error;
    this->last_setpoint = setpoint;
    this->last_measured = measured;

    float result = p_term + integral + d_term;

    /* Output limit */
    clamp(result, -output_limit, output_limit);

    return result;
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

float PIDController::get_p() {
    return this->p_gain;
}

float PIDController::get_i() {
    return this->i_gain;
}

float PIDController::get_d() {
    return this->d_gain;
}

void PIDController::set_params(const PIDParams& params) {
    p_gain = params.p_gain;
    i_gain = params.i_gain;
    d_gain = params.d_gain;
}

void PIDController::integral_reset() {
    integral = 0;
}
