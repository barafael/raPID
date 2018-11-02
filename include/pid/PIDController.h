#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#include "../../include/ArduinoMock.h"

#include "../util.h"

typedef enum { ERROR, SETPOINT, FEEDBACK } derivative_type;
typedef enum { NONE, MOVING_AVERAGE, LOWPASS } filter_type;

typedef float (*filter_func)(float param);

typedef struct {
        bool enabled;

        float p_gain;
        float i_gain;
        float d_gain;

        float integral;
        float integral_limit;

        float derivative;

        derivative_type d_type;

        /* For derivative-on-error */
        float last_error;

        /* For derivative-on-setpoint */
        float last_setpoint;

        /* For derivative-on-measured */
        float last_measured;

        float output_limit;

        uint64_t last_time;

        filter_type derivative_filter_type;

        filter_func deriv_filter;
        bool deriv_filter_enabled;
} pid_controller_t;

pid_controller_t pid_controller_init(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit);

/* En/Disable Passthrough of setpoint */
void pid_set_enabled(pid_controller_t *self, bool enable);

float pid_compute(pid_controller_t *self, float measured, float setpoint);

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

void pid_set_filter(pid_controller_t *self, filter_func filter);
void pid_set_enable_derivative_filter(pid_controller_t *self, bool enable);


#endif // PID_CONTROLLER_H
