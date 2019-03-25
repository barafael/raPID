#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "../../include/Mock.h"

#include "../../include/filter/complementary_filter.h"
#include "../../include/filter/moving_average_filter.h"

#include "../util.h"

typedef enum { PID_UNINITIALIZED, PID_INITIALIZED } pid_status;

#define THROTTLE_LOW_CUTOFF 25

typedef enum { ERROR, SETPOINT/*, FEEDBACK*/ } derivative_type;
#define FILTER_TYPE NONE

typedef struct {
    bool enabled;

    float p_gain;
    float i_gain;
    float d_gain;

    float integral;
    float integral_limit;

    derivative_type d_type;

    /* For derivative-on-error */
    float last_error;

    /* For derivative-on-setpoint */
    float last_setpoint;

    float output_limit;

    //implements: GLOBAL_timestamp_type
    uint64_t last_time;

#ifndef FILTER_TYPE
#else
#if FILTER_TYPE == NONE
#else
#if FILTER_TYPE == MOVING_AVERAGE
    moving_average_t moving_average_filter;
#else
#error "Undefined filter type!"
#endif
#endif
#endif

} pid_controller_t;

pid_controller_t pid_controller_init(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit);

/* En/Disable Passthrough of setpoint */
void pid_set_enabled(pid_controller_t *self, bool enable);

float pid_compute(pid_controller_t *self, float measured, float setpoint);

void pid_integral_reset(pid_controller_t *self);

void pid_set_p(pid_controller_t *self, float _p_gain);
void pid_set_i(pid_controller_t *self, float _i_gain);
void pid_set_d(pid_controller_t *self, float _d_gain);

void pid_set_params(pid_controller_t *self,
        float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit);

float pid_get_p(pid_controller_t *self);
float pid_get_i(pid_controller_t *self);
float pid_get_d(pid_controller_t *self);

void pid_integral_reset(pid_controller_t *self);

void pid_set_derivative_type(pid_controller_t *self, derivative_type type);

void pid_set_integral_limit(pid_controller_t *self, float limit);
void pid_set_output_limit(pid_controller_t *self, float limit);

#endif // PID_CONTROLLER_H
