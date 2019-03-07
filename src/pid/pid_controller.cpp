#include "../../include/pid/pid_controller.h"

/*@ requires PID_limit_bounds: 0 <= \abs(integral_limit) < \abs(output_limit);
    requires PID_integral_limit_positive: PID_limit_bounds: integral_limit >= 0;
    requires PID_output_limit_positive: PID_limit_bounds: output_limit >= 0;

    assigns \nothing;

    ensures \result.enabled == 1;

    ensures \result.p_gain == p_gain;
    ensures \result.i_gain == i_gain;
    ensures \result.d_gain == d_gain;

    ensures \result.integral == 0;

    ensures \result.d_type == ERROR;

    ensures \result.last_error == 0;

    ensures \result.last_setpoint == 0;

    ensures PID_integral_limit_positive: PID_limit_bounds: \result.integral_limit == \abs(integral_limit);
    ensures PID_output_limit: PID_limit_bounds: \result.output_limit == \abs(output_limit);

    ensures \result.last_time == 0;

    ensures PID_limit_bounds: 0.0f <= \result.integral_limit < \result.output_limit;
    
    ensures pid_init_state == PID_INITIALIZED;
*/
//implements: PID_init
pid_controller_t pid_controller_init(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit) {
#ifndef FILTER_TYPE
#else
#if FILTER_TYPE == NONE
#else
#if FILTER_TYPE == MOVING_AVERAGE
    moving_average_t mavg_filter = init_moving_average_filter();
#else
#error "Undefined filter type!"
#endif
#endif
#endif
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite(integral_limit); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite(output_limit); */
    pid_controller_t controller = {
        .enabled = true,

        .p_gain = p_gain,
        .i_gain = i_gain,
        .d_gain = d_gain,

        .integral       = 0.0f,
        //implements: PID_integral_limit_positive
        .integral_limit = abs(integral_limit),

        .d_type = ERROR,

        /* For derivative-on-error */
        .last_error = 0.0f,

        /* For derivative-on-setpoint */
        .last_setpoint = 0.0f,

        //implements: PID_output_limit_positive
        .output_limit = abs(output_limit),

        .last_time = mock_micros(),
#ifndef FILTER_TYPE
#else
#if FILTER_TYPE == NONE
#else
#if FILTER_TYPE == MOVING_AVERAGE
        .moving_average_filter = mavg_filter,
#else
#error "Undefined filter type!"
#endif
#endif
#endif
    };
    //@ assert PID_output_limit_positive: controller.output_limit >= 0.0f;
    //@ assert PID_integral_limit_positive: controller.integral_limit >= 0.0f;
    //@ assert PID_limit_bounds: controller.integral_limit <= controller.output_limit;
    //@ requires PID_init: pid_init_state == PID_INITIALIZED;
    return controller;
}

/* En/Disable Passthrough of setpoint */
/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 ensures self->enabled == enable;
*/
void pid_set_enabled(pid_controller_t *self, bool enable) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->last_time); */
    if (enable) {
        //implement: PID_enable_must_set_time
        self->last_time = mock_micros();
    } else {
        pid_integral_reset(self);
        /*@ assert PID_disable_must_reset: self->integral == 0.0f; */

        /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->last_error); */
        self->last_error = 0.0f;
        /*@ assert PID_disable_must_reset: self->last_error == 0.0f; */

        /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->last_setpoint); */
        self->last_setpoint = 0.0f;
        /*@ assert PID_disable_must_reset: self->last_setpoint == 0.0f; */
    }
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->enabled); */
    self->enabled = enable;
}

/*@
   requires \valid(self);
   requires PID_limit_bounds: 0 <= self->integral_limit <= self->output_limit;
   requires \is_finite(measured);
   requires \is_finite(setpoint);
   requires PID_init: pid_init_state == PID_INITIALIZED;

   assigns PID_not_constant_rate: self->last_time;
   assigns self->last_error;
   assigns self->last_setpoint;
   assigns self->integral;

   //ensures \valid(\old(self)) ==> \valid(self);

   ensures PID_output_limit: -self->output_limit <= \result <= self->output_limit;

   behavior PID_disabled_setpoint_passthrough:
     assumes self->enabled == false;
     ensures PID_not_constant_rate: self->last_time == \old(self->last_time);
     ensures self->last_error == \old(self->last_error);
     ensures self->last_setpoint == \old(self->last_setpoint);
     ensures \result == setpoint;

   behavior enabled:
     assumes self->enabled;
     ensures PID_output_limit: -self->output_limit <= \result <= self->output_limit;
     // this is an assumption about the loop rate which is correct
     //ensures \old(self->last_time) < \at(self->last_time, Post);
   complete behaviors;
   disjoint behaviors;
*/
float pid_compute(pid_controller_t *self, float measured, float setpoint) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->enabled); */
    if (!self->enabled) {
        return setpoint;
    }

    //implements GLOBAL_timestamp_type
    uint64_t now = mock_micros();
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->last_time); */
    // unverifyable: frama-c knows nothing about loop rate and actual milliseconds

    // assert PID_deltatime_positive: now >= self->last_time;
    // does not work because of assumption that looptime is less than UINT64_MAX microseconds.

    /* If there is overflow, the elapsed time is still correct
     * The calculation overflows just like the timer
     * Assumption: loop time is shorter than the time it takes where micros() overflows.
     * If loop time were lower at any time, there'd be much larger problems. */
    //implements: PID_not_constant_rate
    uint64_t elapsed_time = now - self->last_time;
    // if loop time less than timer resolution, then elapsed is 0.
    if (elapsed_time == 0) {
        elapsed_time = 1;
    }
    /*@ assert PID_deltatime_positive: elapsed_time != 0; */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)(measured - setpoint));*/
    //implements: PID_error_calc
    float error = measured - setpoint;

    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->p_gain); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)(self->p_gain * error)); */
    float p_term = self->p_gain * error;
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->integral); */
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->integral); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)((float)elapsed_time * error)); */
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->i_gain); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)((float)((float)elapsed_time * error) * self->i_gain)); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)(self->integral + (float)((float)((float)elapsed_time * error) * self->i_gain))); */
    //implements: PID_not_constant_rate
    self->integral += elapsed_time * error * self->i_gain;
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->integral); */
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->integral_limit); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite(self->integral_limit); */

  /* Integral windup clamp */
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->integral); */
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->integral_limit); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite(self->integral_limit); */

    clamp(self->integral, -self->integral_limit, self->integral_limit);
    //@ assert PID_integral_limit: -self->integral_limit <= self->integral <= self->integral_limit;

    float d_term = 0.0f;

    //@ assert elapsed_not_zero: elapsed_time != 0;
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->d_type); */
    switch (self->d_type) {
        /* Derivative term on error */
        case ERROR:
            /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->last_error); */
            /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)(error - self->last_error));
            */
            /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)((float)(error - self->last_error) / (float)elapsed_time)); */
            /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->d_gain); */
            /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)((float)((float)(error - self->last_error) /
(float)elapsed_time) * self->d_gain)); */
            //implements: PID_not_constant_rate
            d_term = ((error - self->last_error) / elapsed_time) * self->d_gain;
            break;

            /* Derivative term on setpoint */
        case SETPOINT:
            /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->last_setpoint); */
            /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)(setpoint - self->last_setpoint)); */
            /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)((float)(setpoint - self->last_setpoint) / (float)elapsed_time)); */
            /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->d_gain); */
            /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)((float)((float)(setpoint - self->last_setpoint) / (float)elapsed_time) * self->d_gain)); */
            //implements: PID_not_constant_rate
            d_term = ((setpoint - self->last_setpoint) / elapsed_time) * self->d_gain;
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
#error "Undefined filter type!"
#endif
#endif
#endif
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->last_time); */
    self->last_time = now;
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->last_error); */

    self->last_error    = error;
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->last_setpoint); */

    self->last_setpoint = setpoint;
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->integral); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)(p_term + self->integral)); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite((float)((float)(p_term + self->integral) + d_term)); */
    float result = p_term + self->integral + d_term;
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->output_limit); */
    /*@ assert GLOBAL_undef_behavior: is_nan_or_infinite: \is_finite(self->output_limit); */

    /* Output limit */
    clamp(result, -self->output_limit, self->output_limit);
    //@ assert PID_output_limit: -self->output_limit <= result <= self->output_limit;
    return result;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 ensures self->p_gain == _p_gain;
*/
void pid_set_p(pid_controller_t *self, float _p_gain) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->p_gain); */
    self->p_gain = _p_gain;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->i_gain == _i_gain;
*/
void pid_set_i(pid_controller_t *self, float _i_gain) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->i_gain); */
    self->i_gain = _i_gain;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->d_gain == _d_gain;
*/
void pid_set_d(pid_controller_t *self, float _d_gain) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->d_gain); */
    self->d_gain = _d_gain;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 requires PID_integral_limit: integral_limit <= output_limit;

 ensures self->p_gain == p_gain;
 ensures self->i_gain == i_gain;
 ensures self->d_gain == d_gain;

 ensures PID_integral_limit: self->integral_limit == integral_limit;
 ensures self->output_limit == output_limit; */
void pid_set_params(pid_controller_t *self,
        float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->p_gain); */
    self->p_gain = p_gain;

    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->i_gain); */
    self->i_gain = i_gain;

    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->d_gain); */
    self->d_gain = d_gain;

    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->integral_limit); */
    self->integral_limit = integral_limit;

    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->output_limit); */
    self->output_limit   = output_limit;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->p_gain;
*/
float pid_get_p(pid_controller_t *self) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->p_gain); */
    return self->p_gain;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->i_gain; */
float pid_get_i(pid_controller_t *self) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->i_gain); */
    return self->i_gain;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->d_gain;
*/
float pid_get_d(pid_controller_t *self) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(&self->d_gain); */
    return self->d_gain;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->integral == 0.0;
*/
void pid_integral_reset(pid_controller_t *self) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->integral); */
    self->integral = 0.0f;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->integral_limit == limit;
*/
void pid_set_integral_limit(pid_controller_t *self, float limit) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->integral_limit); */
    self->integral_limit = limit;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->output_limit == limit;
*/
void pid_set_output_limit(pid_controller_t *self, float limit) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->output_limit); */
    self->output_limit = limit;
}

/*@
 requires \valid(self);
 requires PID_init: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->d_type == type;
*/
void pid_set_derivative_type(pid_controller_t *self, derivative_type type) {
    /*@ assert GLOBAL_undef_behavior: mem_access: \valid(&self->d_type); */
    self->d_type = type;
}

