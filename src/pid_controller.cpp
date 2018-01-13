#include "../interface/pid_controller.h"

#define clamp(value, low, high) ((value) = ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value))))

pid_controller::pid_controller(const float p_coeff, const float i_coeff, const float d_coeff,
        const float integral_limit, const float output_limit)
    : p_coeff { p_coeff }
    , i_coeff { i_coeff }
    , d_coeff { d_coeff }

    /* TODO test if there are problems in the first couple loops after init
     * (because of 0 init for derivtive and integral) */
    , integral      { 0 }
    , integral_limit { integral_limit }

    , derivative    { 0 }
    , last_error    { 0 }
    , last_setpoint { 0 }

    , output_limit   { output_limit }

    , last_time     { 0 }
{

}

/* TODO: handle overflows for 'now' using rollover or somesuch. */
float pid_controller::compute(const uint64_t now, const float measured, const float setpoint) {

    uint32_t elapsed_time = now - last_time;

    float error = measured - setpoint;

    float result = 0.0;

    /* Give me some P! */
    /* Proportional term */
    result += this->p_coeff * error;

    /* Give me some I! */
    /* Integral term */
    this->integral += elapsed_time * error * this->i_coeff;
    /* Integral windup limit */
    this->integral = this->integral > this->integral_limit ? this->integral_limit : this->integral;
    this->integral = this->integral < this->integral_limit * -1 ? this->integral_limit * -1 : this->integral;

    result += this->integral;

    /* Give me some D! */
    /* Derivative term on error */
    result += ((error - last_error) / elapsed_time) * this->d_coeff;

    /* Derivative term on input */
    // result += (setpoint - last_setpoint) / elapsed_time;

    /* Output limit */
    clamp(result, -output_limit, output_limit);

    this->last_error    = error;
    this->last_time     = now;
    this->last_setpoint = setpoint;

    return result;
}

pid_controller *pid_controller::set_p(const float kp) {
    this->p_coeff = p_coeff;
    return this;
}

pid_controller *pid_controller::set_i(const float ki) {
    this->i_coeff = i_coeff;
    return this;
}

pid_controller *pid_controller::set_d(const float kd) {
    this->d_coeff = d_coeff;
    return this;
}

pid_controller *pid_controller::integral_reset() {
    integral = 0;
    return this;
}
