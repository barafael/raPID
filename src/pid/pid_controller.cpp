#include "../../include/pid/pid_controller.h"

/*@ requires PIDlimitBounds: 0 <= \abs(integral_limit) < \abs(output_limit);
    requires PIDintegralLimitPositive: PIDlimitBounds: integral_limit >= 0;
    requires PIDoutputLimitPositive: PIDlimitBounds: output_limit >= 0;

    assigns \nothing;

    ensures \result.enabled == 1;

    ensures \result.p_gain == p_gain;
    ensures \result.i_gain == i_gain;
    ensures \result.d_gain == d_gain;

    ensures \result.integral == 0;

    ensures \result.d_type == ERROR;

    ensures \result.last_error == 0;

    ensures \result.last_setpoint == 0;

    ensures PIDintegralLimitPositive: PIDlimitBounds: \result.integral_limit == \abs(integral_limit);
    ensures PIDoutputLimit: PIDlimitBounds: \result.output_limit == \abs(output_limit);

    ensures \result.last_time == 0;

    ensures PIDlimitBounds: 0.0f <= \result.integral_limit < \result.output_limit;
    
    ensures pid_init_state == PID_INITIALIZED;
*/
//implements: PIDinit
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
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite(integral_limit); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite(output_limit); */
    pid_controller_t controller = {
        .enabled = true,

        .p_gain = p_gain,
        .i_gain = i_gain,
        .d_gain = d_gain,

        .integral       = 0.0f,
        //implements: PIDintegralLimitPositive
        .integral_limit = abs(integral_limit),

        .d_type = ERROR,

        /* For derivative-on-error */
        .last_error = 0.0f,

        /* For derivative-on-setpoint */
        .last_setpoint = 0.0f,

        //implements: PIDoutputLimitPositive
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
    //@ assert PIDoutputLimitPositive: controller.output_limit >= 0.0f;
    //@ assert PIDintegralLimitPositive: controller.integral_limit >= 0.0f;
    //@ assert PIDlimitBounds: controller.integral_limit <= controller.output_limit;
    //@ requires PIDinit: pid_init_state == PID_INITIALIZED;
    return controller;
}

/* En/Disable Passthrough of setpoint */
/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 ensures self->enabled == enable;
*/
void pid_set_enabled(pid_controller_t *self, bool enable) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->last_time); */
    if (enable) {
        self->last_time = mock_micros();
    } else {
        pid_integral_reset(self);
        /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->last_error); */
        self->last_error = 0.0f;
        /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->last_setpoint); */
        self->last_setpoint = 0.0f;
    }
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->enabled); */
    self->enabled = enable;
}

/*@
   requires \valid(self);
   requires PIDlimitBounds: 0 <= self->integral_limit <= self->output_limit;
   requires \is_finite(measured);
   requires \is_finite(setpoint);
   requires PIDinit: pid_init_state == PID_INITIALIZED;

   assigns PIDnotConstantRate: self->last_time;
   assigns self->last_error;
   assigns self->last_setpoint;
   assigns self->integral;

   //ensures \valid(\old(self)) ==> \valid(self);

   ensures PIDoutputLimit: -self->output_limit <= \result <= self->output_limit;

   behavior PIDdisabledSetpointPassthrough:
     assumes self->enabled == false;
     ensures PIDnotConstantRate: self->last_time == \old(self->last_time);
     ensures self->last_error == \old(self->last_error);
     ensures self->last_setpoint == \old(self->last_setpoint);
     ensures \result == setpoint;

   behavior enabled:
     assumes self->enabled;
     ensures PIDoutputLimit: -self->output_limit <= \result <= self->output_limit;
     // this is an assumption about the loop rate which is correct
     //ensures \old(self->last_time) < \at(self->last_time, Post);
   complete behaviors;
   disjoint behaviors;
*/
float pid_compute(pid_controller_t *self, float measured, float setpoint) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->enabled); */
    if (!self->enabled) {
        return setpoint;
    }

    //implements GLOBALtimestampType
    uint64_t now = mock_micros();
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->last_time); */
    // unverifyable: frama-c knows nothing about loop rate and actual milliseconds

    // assert PIDdeltatimePositive: now >= self->last_time;
    // does not work because of assumption that looptime is less than UINT64_MAX microseconds.

    /* If there is overflow, the elapsed time is still correct
     * The calculation overflows just like the timer
     * Assumption: loop time is shorter than the time it takes where micros() overflows.
     * If loop time were lower at any time, there'd be much larger problems. */
    //implements: PIDnotConstantRate
    uint64_t elapsed_time = now - self->last_time;
    // if loop time less than timer resolution, then elapsed is 0.
    if (elapsed_time == 0) {
        elapsed_time = 1;
    }
    /*@ assert PIDdeltatimeNonZero: elapsed_time != 0; */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)(measured - setpoint));*/
    //implements: PIDerrorCalc
    float error = measured - setpoint;

    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->p_gain); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)(self->p_gain * error)); */
    float p_term = self->p_gain * error;
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->integral); */
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->integral); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)((float)elapsed_time * error)); */
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->i_gain); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)((float)((float)elapsed_time * error) * self->i_gain)); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)(self->integral + (float)((float)((float)elapsed_time * error) * self->i_gain))); */
    //implements: PIDnotConstantRate
    self->integral += elapsed_time * error * self->i_gain;
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->integral); */
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->integral_limit); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite(self->integral_limit); */

  /* Integral windup clamp */
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->integral); */
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->integral_limit); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite(self->integral_limit); */

    clamp(self->integral, -self->integral_limit, self->integral_limit);
    //@ assert PIDintegralLimit: -self->integral_limit <= self->integral <= self->integral_limit;

    float d_term = 0.0f;

    //@ assert elapsed_not_zero: elapsed_time != 0;
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->d_type); */
    switch (self->d_type) {
        /* Derivative term on error */
        case ERROR:
            /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->last_error); */
            /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)(error - self->last_error));
            */
            /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)((float)(error - self->last_error) / (float)elapsed_time)); */
            /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->d_gain); */
            /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)((float)((float)(error - self->last_error) /
(float)elapsed_time) * self->d_gain)); */
            //implements: PIDnotConstantRate
            d_term = ((error - self->last_error) / elapsed_time) * self->d_gain;
            break;

            /* Derivative term on setpoint */
        case SETPOINT:
            /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->last_setpoint); */
            /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)(setpoint - self->last_setpoint)); */
            /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)((float)(setpoint - self->last_setpoint) / (float)elapsed_time)); */
            /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->d_gain); */
            /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)((float)((float)(setpoint - self->last_setpoint) / (float)elapsed_time) * self->d_gain)); */
            //implements: PIDnotConstantRate
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
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->last_time); */
    self->last_time = now;
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->last_error); */

    self->last_error    = error;
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->last_setpoint); */

    self->last_setpoint = setpoint;
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->integral); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)(p_term + self->integral)); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite((float)((float)(p_term + self->integral) + d_term)); */
    float result = p_term + self->integral + d_term;
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->output_limit); */
    /*@ assert GLOBALundefBehavior: is_nan_or_infinite: \is_finite(self->output_limit); */

    /* Output limit */
    clamp(result, -self->output_limit, self->output_limit);
    //@ assert PIDoutputLimit: -self->output_limit <= result <= self->output_limit;
    return result;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 ensures self->p_gain == _p_gain;
*/
void pid_set_p(pid_controller_t *self, float _p_gain) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->p_gain); */
    self->p_gain = _p_gain;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->i_gain == _i_gain;
*/
void pid_set_i(pid_controller_t *self, float _i_gain) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->i_gain); */
    self->i_gain = _i_gain;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->d_gain == _d_gain;
*/
void pid_set_d(pid_controller_t *self, float _d_gain) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->d_gain); */
    self->d_gain = _d_gain;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 requires PIDintegralLimit: integral_limit <= output_limit;

 ensures self->p_gain == p_gain;
 ensures self->i_gain == i_gain;
 ensures self->d_gain == d_gain;

 ensures PIDintegralLimit: self->integral_limit == integral_limit;
 ensures self->output_limit == output_limit; */
void pid_set_params(pid_controller_t *self,
        float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->p_gain); */
    self->p_gain = p_gain;

    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->i_gain); */
    self->i_gain = i_gain;

    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->d_gain); */
    self->d_gain = d_gain;

    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->integral_limit); */
    self->integral_limit = integral_limit;

    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->output_limit); */
    self->output_limit   = output_limit;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->p_gain;
*/
float pid_get_p(pid_controller_t *self) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->p_gain); */
    return self->p_gain;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->i_gain; */
float pid_get_i(pid_controller_t *self) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->i_gain); */
    return self->i_gain;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->d_gain;
*/
float pid_get_d(pid_controller_t *self) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid_read(&self->d_gain); */
    return self->d_gain;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->integral == 0.0;
*/
void pid_integral_reset(pid_controller_t *self) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->integral); */
    self->integral = 0.0f;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->integral_limit == limit;
*/
void pid_set_integral_limit(pid_controller_t *self, float limit) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->integral_limit); */
    self->integral_limit = limit;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->output_limit == limit;
*/
void pid_set_output_limit(pid_controller_t *self, float limit) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->output_limit); */
    self->output_limit = limit;
}

/*@
 requires \valid(self);
 requires PIDinit: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->d_type == type;
*/
void pid_set_derivative_type(pid_controller_t *self, derivative_type type) {
    /*@ assert GLOBALundefBehavior: mem_access: \valid(&self->d_type); */
    self->d_type = type;
}

