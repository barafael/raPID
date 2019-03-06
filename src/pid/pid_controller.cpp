#include "../../include/pid/pid_controller.h"

/*@ requires PIDlimitBounds: 0 <= \abs(integral_limit) < \abs(output_limit);
    requires PIDlimitBounds: integral_limit >= 0;
    requires PIDlimitBounds: output_limit >= 0;

    assigns \nothing;

    ensures \result.enabled == 1;

    ensures \result.p_gain == p_gain;
    ensures \result.i_gain == i_gain;
    ensures \result.d_gain == d_gain;

    ensures PIDintegralLimit: \result.integral == 0;

    ensures \result.d_type == ERROR;

    ensures \result.last_error == 0;

    ensures \result.last_setpoint == 0;

    ensures PIDlimitBounds: \result.integral_limit == \abs(integral_limit);
    ensures PIDlimitBounds: \result.output_limit == \abs(output_limit);

    ensures \result.last_time == 0;

    ensures PIDlimitBounds: 0.0f <= \result.integral_limit < \result.output_limit;
    
    ensures pid_init_state == PID_INITIALIZED;
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
#error "Undefined filter type!"
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
    //@ assert PIDlimitBounds: controller.output_limit >= 0.0f;
    //@ assert PIDlimitBounds: controller.integral_limit >= 0.0f;
    //@ assert PIDlimitBounds: controller.integral_limit <= controller.output_limit;
    //@ requires PIDinitialization: pid_init_state == PID_INITIALIZED;
    return controller;
}

/* En/Disable Passthrough of setpoint */
/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 ensures self->enabled == enable;
*/
void pid_set_enabled(pid_controller_t *self, bool enable) {
    if (enable) {
        self->last_time = mock_micros();
    } else {
        pid_integral_reset(self);
        self->last_error = 0.0f;
        self->last_setpoint = 0.0f;
    }
    self->enabled = enable;
}

/*@
   requires \valid(self);
   requires PIDlimitBounds: 0 <= self->integral_limit <= self->output_limit;
   requires \is_finite(measured);
   requires \is_finite(setpoint);
   requires PIDinitialization: pid_init_state == PID_INITIALIZED;

   assigns self->last_time;
   assigns self->last_error;
   assigns self->last_setpoint;
   assigns self->integral;

   //ensures \valid(\old(self)) ==> \valid(self);

   ensures PIDboundedResult: -self->output_limit <= \result <= self->output_limit;

   behavior PIDdisabledSetpointPassthrough:
     assumes self->enabled == false;
     ensures self->last_time == \old(self->last_time);
     ensures self->last_error == \old(self->last_error);
     ensures self->last_setpoint == \old(self->last_setpoint);
     ensures \result == setpoint;

   behavior enabled:
     assumes self->enabled;
     ensures PIDboundedResult: -self->output_limit <= \result <= self->output_limit;
     // this is an assumption about the loop rate which is correct (otherwise a loop iteration would take hours)
     // but frama-c cannot take that into account
     //ensures \old(self->last_time) < \at(self->last_time, Post);
   complete behaviors;
   disjoint behaviors;
*/
float pid_compute(pid_controller_t *self, float measured, float setpoint) {
    if (!self->enabled) {
        return setpoint;
    }

    uint64_t now = mock_micros();
    // unverifyable: frama-c knows nothing about loop rate and actual milliseconds
    // When loop rate is HOURS, this might malfunction, but honestly...

    // assert PIDdeltatimePositive: now >= self->last_time;
    // does not work because of assumption that looptime is less than UINT64_MAX
    // microseconds.

    /* If there is overflow, the elapsed time is still correct
     * The calculation overflows just like the timer
     * Assumption: loop time is shorter than the time it takes where micros() overflows.
     * If loop time were lower at any time, there'd be much larger problems. */
    uint64_t elapsed_time = now - self->last_time;
    /* if loop time less than 1 microsecond, then elapsed is 0.
     * Actual resulution of micros() timer is less, but this loop time would need to be 3 orders of magnitude larger than the actual looprate. */
    if (elapsed_time == 0) {
        elapsed_time = 1;
    }
    //@ assert PIDdeltatimeNonZero: elapsed_time != 0;

    float error = measured - setpoint;

    float p_term = self->p_gain * error;

    self->integral += elapsed_time * error * self->i_gain;
    /* Integral windup clamp */
    clamp(self->integral, -self->integral_limit, self->integral_limit);
    //@ assert PIDintegralLimit: -self->integral_limit <= self->integral <= self->integral_limit;

    float d_term = 0.0f;

    //@ assert elapsed_not_zero: elapsed_time != 0;
    switch (self->d_type) {
        /* Derivative term on error */
        case ERROR:
            d_term = ((error - self->last_error) / elapsed_time) * self->d_gain;
            break;

            /* Derivative term on setpoint */
        case SETPOINT:
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

    self->last_time = now;

    self->last_error    = error;
    self->last_setpoint = setpoint;

    float result = p_term + self->integral + d_term;

    /* Output limit */
    clamp(result, -self->output_limit, self->output_limit);
    //@ assert PIDboundedResult: -self->output_limit <= result <= self->output_limit;

    return result;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 ensures self->p_gain == _p_gain;
*/
void pid_set_p(pid_controller_t *self, float _p_gain) {
    self->p_gain = _p_gain;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->i_gain == _i_gain;
*/
void pid_set_i(pid_controller_t *self, float _i_gain) {
    self->i_gain = _i_gain;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->d_gain == _d_gain;
*/
void pid_set_d(pid_controller_t *self, float _d_gain) {
    self->d_gain = _d_gain;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
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
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->p_gain;
*/
float pid_get_p(pid_controller_t *self) {
    return self->p_gain;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->i_gain;
*/
float pid_get_i(pid_controller_t *self) {
    return self->i_gain;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures \result == self->d_gain;
*/
float pid_get_d(pid_controller_t *self) {
    return self->d_gain;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->integral == 0.0;
*/
void pid_integral_reset(pid_controller_t *self) {
    self->integral = 0.0f;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->integral_limit == limit;
*/
void pid_set_integral_limit(pid_controller_t *self, float limit) {
    self->integral_limit = limit;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->output_limit == limit;
*/
void pid_set_output_limit(pid_controller_t *self, float limit) {
    self->output_limit = limit;
}

/*@
 requires \valid(self);
 requires PIDinitialization: pid_init_state == PID_INITIALIZED;
 //ensures \valid(\old(self)) ==> \valid(self);
 ensures self->d_type == type;
*/
void pid_set_derivative_type(pid_controller_t *self, derivative_type type) {
    self->d_type = type;
}
