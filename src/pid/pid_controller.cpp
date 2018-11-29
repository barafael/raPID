#include "../../include/pid/pid_controller.h"

/*@ requires \abs(integral_limit) < \abs(output_limit);
    assigns \nothing;

    ensures \result.enabled == 1;

    ensures \result.p_gain == p_gain;
    ensures \result.i_gain == i_gain;
    ensures \result.d_gain == d_gain;

    ensures \result.integral == 0;

    ensures \result.d_type == ERROR;

    ensures \result.last_error == 0;

    ensures \result.last_setpoint == 0;

    ensures \result.last_measured == 0;

    ensures \result.integral_limit == \abs(integral_limit);
    ensures \result.output_limit == \abs(output_limit);

    ensures \result.last_time == 0;

    ensures 0.0f <= \result.integral_limit < \result.output_limit;
*/
pid_controller_t pid_controller_init(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit) {
#ifndef FILTER_TYPE
#else
#if FILTER_TYPE == NONE
#else
#if FILTER_TYPE == MOVING_AVERAGE
    moving_average_t mavg_filter = init_moving_average_filter();
#else
#if FILTER_TYPE == COMPLEMENTARY
    complementary_filter_t comp_filter = init_complementary_filter(0.5);
#else
#error "Undefined filter type!"
#endif
#endif
#endif
#endif

    pid_controller_t controller = {
        .enabled = true,

        .p_gain = p_gain,
        .i_gain = i_gain,
        .d_gain = d_gain,

        .integral       = 0.0f,
        .integral_limit = abs(integral_limit),

        .d_type = ERROR,

        /* For derivative-on-error */
        .last_error = 0.0f,

        /* For derivative-on-setpoint */
        .last_setpoint = 0.0f,

        /* For derivative-on-measured */
        .last_measured = 0.0f,

        .output_limit = abs(output_limit),

        .last_time = 0,
#ifndef FILTER_TYPE
#else
#if FILTER_TYPE == NONE
#else
#if FILTER_TYPE == MOVING_AVERAGE
        .moving_average_filter = mavg_filter,
#else
#if FILTER_TYPE == COMPLEMENTARY
        .complementary_filter = comp_filter,
#else
#error "Undefined filter type!"
#endif
#endif
#endif
#endif
    };
    return controller;
}

/* En/Disable Passthrough of setpoint */
/*@
 requires \valid(self);
 ensures \valid(\old(self)) ==> \valid(self);
 ensures self->enabled == enable; */
void pid_set_enabled(pid_controller_t *self, bool enable) {
    self->enabled = enable;
}

/*@
   requires \valid(self);
   requires self->output_limit > 0.0f;

   assigns self->last_time;
   assigns self->last_error;
   assigns self->last_setpoint;
   assigns self->last_measured;

   ensures \valid(\old(self)) ==> \valid(self);

   behavior disabled:
     assumes self->enabled == false;
     ensures self->last_time == \old(self->last_time);
     ensures self->last_error == \old(self->last_error);
     ensures self->last_setpoint == \old(self->last_setpoint);
     ensures self->last_measured == \old(self->last_measured);
     ensures \result == setpoint;

   behavior enabled:
     assumes self->enabled;
     ensures \result <= self->output_limit;
     ensures \old(self->last_time) < \at(self->last_time, Post);
   complete behaviors;
   disjoint behaviors;
*/
float pid_compute(pid_controller_t *self, float measured, float setpoint) {
    if (!self->enabled) {
        return setpoint;
    }

    uint64_t now = mock_micros();

    /* If there is overflow, the elapsed time is still correct
     * The calculation overflows just like the timer */
    uint64_t elapsed_time = now - self->last_time;
    //@ assert elapsed_time > 0;

    float error = measured - setpoint;

    float p_term = self->p_gain * error;

    self->integral += elapsed_time * error * self->i_gain;
    /* Integral windup clamp */
    clamp(self->integral, -self->integral_limit, self->integral_limit);
    //@ assert -self->integral_limit <= self->integral <= self->integral_limit;

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
    //@ assert \is_finite(d_term);

#ifndef FILTER_TYPE
#else
#if FILTER_TYPE == NONE
#else
#if FILTER_TYPE == MOVING_AVERAGE
    d_term = moving_average_next(&self->moving_average_filter, d_term);
#else
#if FILTER_TYPE == COMPLEMENTARY
    d_term = complementary_next(&self->complementary_filter, d_term);
#else
#error "Undefined filter type!"
#endif
#endif
#endif
#endif

    self->last_time = now;

    self->last_error    = error;
    self->last_setpoint = setpoint;
    self->last_measured = measured;

    float result = p_term + self->integral + d_term;

    /* Output limit */
    clamp(result, -self->output_limit, self->output_limit);
    //@ assert -self->output_limit <= result <= self->output_limit;

    return result;
}

/*@
 requires \valid(self);
 ensures \valid(\old(self)) ==> \valid(self);
 ensures self->p_gain == _p_gain;
*/
void pid_set_p(pid_controller_t *self, float _p_gain) {
    self->p_gain = _p_gain;
}

/*@
 requires \valid(self);
 ensures \valid(\old(self)) ==> \valid(self);
 ensures self->i_gain == _i_gain;
*/
void pid_set_i(pid_controller_t *self, float _i_gain) {
    self->i_gain = _i_gain;
}

/*@
 requires \valid(self);
 ensures \valid(\old(self)) ==> \valid(self);
 ensures self->d_gain == _d_gain;
*/
void pid_set_d(pid_controller_t *self, float _d_gain) {
    self->d_gain = _d_gain;
}

/*@
 requires \valid(self);
 // bug in spec and still proven! > instead of <
 requires integral_limit <= output_limit;

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
    self->output_limit   = output_limit;
}

/*@
 requires \valid(self);
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->p_gain;
*/
float pid_get_p(pid_controller_t *self) {
    return self->p_gain;
}

/*@
 requires \valid(self);
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->i_gain;
*/
float pid_get_i(pid_controller_t *self) {
    return self->i_gain;
}

/*@
 requires \valid(self);
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->d_gain;
*/
float pid_get_d(pid_controller_t *self) {
    return self->d_gain;
}

/*@
 requires \valid(self);
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->integral == 0.0;
*/
void pid_integral_reset(pid_controller_t *self) {
    self->integral = 0.0f;
}

/*@
 requires \valid(self);
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->integral_limit == limit;
*/
void pid_set_integral_limit(pid_controller_t *self, float limit) {
    self->integral_limit = limit;
}

/*@
 requires \valid(self);
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->output_limit == limit;
*/
void pid_set_output_limit(pid_controller_t *self, float limit) {
    self->output_limit = limit;
}

/*@
 requires \valid(self);
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->d_type == type;
*/
void pid_set_derivative_type(pid_controller_t *self, derivative_type type) {
    self->d_type = type;
}
