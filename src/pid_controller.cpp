#include "../interface/pid_controller.h"

pid_controller::pid_controller(const float kp, const float ki, const float kd,
        const float integral_limit, const float output_limit) {

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->integral_limit = integral_limit;
    this->output_limit   = output_limit;

    /* TODO test if there are problems in the first couple loops after init */
    this->integral      = 0;
    this->derivative    = 0;
    this->last_error    = 0;
    this->last_setpoint = 0;
    this->last_time     = 0;
}

/* TODO: handle overflows for 'now' using rollover or somesuch. */
float pid_controller::compute(const uint64_t now, const float measured, const float setpoint) {

    uint32_t elapsed_time = now - last_time;

    float error = measured - setpoint;

    float result = 0.0;

    /* Proportional term */
    result += this->kp * error;

    /* Integral term */
    this->integral += elapsed_time * error * this->ki;
    /* Integral windup limit */
    this->integral = this->integral > this->integral_limit ? this->integral_limit : this->integral;
    this->integral = this->integral < this->integral_limit * -1 ? this->integral_limit * -1 : this->integral;

    result += this->integral;

    /* Derivative term on error */
    result += ((error - last_error) / elapsed_time) * this->kd;

    /* Derivative term on input */
    // result += (setpoint - last_setpoint) / elapsed_time;

    /* Output limit */
    result = result > this->output_limit ? this->output_limit : result;
    result = result < this->output_limit * -1 ? this->output_limit * -1 : result;

    this->last_error    = error;
    this->last_time     = now;
    this->last_setpoint = setpoint;

    return result;
}

pid_controller *pid_controller::set_p(const float kp) {
    this->kp = kp;
    return this;
}

pid_controller *pid_controller::set_i(const float ki) {
    this->ki = ki;
    return this;
}

pid_controller *pid_controller::set_d(const float kd) {
    this->kd = kd;
    return this;
}
