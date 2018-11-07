#include "../../include/pid/PIDController.h"

/*@
 requires integral_limit <= output_limit;
 assigns \nothing;

 ensures \result.enabled == 1;

 ensures \result.p_gain == p_gain;
 ensures \result.i_gain == i_gain;
 ensures \result.d_gain == d_gain;

 ensures \result.integral == 0;
 ensures \result.integral_limit == integral_limit;

 ensures \result.derivative == 0;

 ensures \result.d_type == ERROR;

 ensures \result.last_error == 0;

 ensures \result.last_setpoint == 0;

 ensures \result.last_measured == 0;

 ensures \result.output_limit == output_limit;

 ensures \result.output_limit == output_limit;

 ensures \result.last_time == 0;

 ensures \result.derivative_filter_type == NONE;

 ensures \result.deriv_filter == 0;

 ensures \result.deriv_filter_enabled == false;
 */
pid_controller_t pid_controller_init(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit) {
    pid_controller_t controller = {
        .enabled = true,

        .p_gain = p_gain,
        .i_gain = i_gain,
        .d_gain = d_gain,

        .integral = 0.0f,
        .integral_limit = integral_limit,

        .derivative = 0.0f,

        .d_type = ERROR,

        /* For derivative-on-error */
        .last_error = 0.0f,

        /* For derivative-on-setpoint */
        .last_setpoint = 0.0f,

        /* For derivative-on-measured */
        .last_measured = 0.0f,

        .output_limit = output_limit,

        .last_time = 0,

        .derivative_filter_type = NONE,

        .deriv_filter = nullptr,
        .deriv_filter_enabled = false,
    };
    return controller;
}


/* En/Disable Passthrough of setpoint */
/*@
 requires \valid(self);
 ensures self->enabled == enable; */
void pid_set_enabled(pid_controller_t *self, bool enable) {
    self->enabled = enable;
}

//requires \valid(self->deriv_filter);
/*@
   requires \valid(self);
   requires self->integral < self->output_limit;
 behavior disabled:
   assumes self->enabled == false;
   assigns \nothing;
   ensures \result == setpoint;

 behavior enabled:
   assumes self->enabled;
   assigns self->last_time;
   assigns self->last_error;
   assigns self->last_setpoint;
   assigns self->last_measured;
   ensures \result <= self->output_limit;
   ensures \old(self->last_time) < self->last_time; // this cannot work, frama-c does not see that micros() is monot. incr.
 complete behaviors;
 disjoint behaviors;
*/
float pid_compute(pid_controller_t *self, float measured, float setpoint) {
    if (!self->enabled) {
        return setpoint;
    }

#ifdef FRAMAC
    uint64_t now = mock_micros();
#else
    uint64_t now = micros();
#endif

    /* If there is overflow, the elapsed time is still correct
     * The calculation overflows just like the timer */
    uint64_t elapsed_time = now - self->last_time;

    float error = measured - setpoint;

    float p_term = self->p_gain * error;

    self->integral += elapsed_time * error * self->i_gain;
    /* Integral windup clamp */
    clamp(self->integral, -self->integral_limit, self->integral_limit);

    float d_term = 0.0f;
    switch (self->d_type) {
        /* Derivative term on error */
        case ERROR:
            d_term = ((error - self->last_error) / elapsed_time) * self->d_gain;
            break;

            /* Derivative term on setpoint */
        case SETPOINT:
            d_term = ((setpoint - self->last_setpoint) / elapsed_time) * self->d_gain;
            break;

            /* Derivative term on measurement */
        case FEEDBACK:
            d_term = ((measured - self->last_measured) / elapsed_time) * self->d_gain;
            break;
    }

    //if (self->deriv_filter_enabled && *(self->deriv_filter) != nullptr) {
    if (self->deriv_filter_enabled) {
        d_term = (self->deriv_filter)(d_term);
    }

    self->last_time     = now;

    self->last_error    = error;
    self->last_setpoint = setpoint;
    self->last_measured = measured;

    float result = p_term + self->integral + d_term;

    /* Output limit */
    clamp(result, -self->output_limit, self->output_limit);

    return result;
}

void pid_set_p(pid_controller_t *self, float _p_gain) {
    self->p_gain = _p_gain;
}

void pid_set_i(pid_controller_t *self, float _i_gain) {
    self->i_gain = _i_gain;
}

void pid_set_d(pid_controller_t *self, float _d_gain) {
    self->d_gain = _d_gain;
}

/*@
 requires integral_limit >= output_limit;
 requires \valid(self);

 ensures self->p_gain == p_gain;
 ensures self->i_gain == i_gain;
 ensures self->d_gain == d_gain;

 ensures self->integral_limit == integral_limit;
 ensures self->output_limit == output_limit;
*/

void pid_set_params(pid_controller_t *self,
        float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit) {
    self->p_gain = p_gain;
    self->i_gain = i_gain;
    self->d_gain = d_gain;

    self->integral_limit = integral_limit;
    self->output_limit = output_limit;
}

float pid_get_p(pid_controller_t *self) {
    return self->p_gain;
}

float pid_get_i(pid_controller_t *self) {
    return self->i_gain;
}

float pid_get_d(pid_controller_t *self) {
    return self->d_gain;
}

void pid_integral_reset(pid_controller_t *self) {
    self->integral = 0.0f;
}

void pid_set_integral_limit(pid_controller_t *self, float limit) {
    self->integral_limit = limit;
}

void pid_set_output_limit(pid_controller_t *self, float limit) {
    self->output_limit = limit;
}

void pid_set_filter(pid_controller_t *self, filter_func filter) {
    self->deriv_filter = filter;
}

void pid_set_enable_derivative_filter(pid_controller_t *self, bool enable) {
    self->deriv_filter_enabled = enable;
}

void pid_set_derivative_type(pid_controller_t *self, derivative_type type) {
    self->d_type = type;
}
