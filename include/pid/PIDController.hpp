#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

#include "Arduino.h"

#include "PIDParams.hpp"
#include "../include/filter/Filter.hpp"
#include "../util.h"

typedef enum { ERROR, SETPOINT, FEEDBACK } derivative_type;
typedef enum { NONE, MOVING_AVERAGE, LOWPASS } filter_type;

class PIDController {
    private:
        bool enabled = true;

        float p_gain;
        float i_gain;
        float d_gain;

        float integral = 0.0f;
        float integral_limit;

        float derivative = 0.0f;

        derivative_type d_type = ERROR;

        /* For derivative-on-error */
        float last_error = 0.0f;

        /* For derivative-on-setpoint */
        float last_setpoint = 0.0f;

        /* For derivative-on-measured */
        float last_measured = 0.0f;

        float output_limit;

        uint64_t last_time = 0.0f;

        filter_type derivative_filter_type = NONE;

        Filter *deriv_filter = nullptr;
        bool deriv_filter_enabled = false;

    public:
        PIDController(float p_gain, float i_gain, float d_gain,
                float integral_limit, float output_limit);
        explicit PIDController(PIDParams& params);

        /* En/Disable Passthrough of setpoint */
        void set_enabled(bool enable);

        float compute(float measured, float setpoint);

        void set_p(float _p_gain);
        void set_i(float _i_gain);
        void set_d(float _d_gain);

        void set_params(const PIDParams& params);

        const float get_p();
        const float get_i();
        const float get_d();

        void integral_reset();

        void set_derivative_type(derivative_type type);

        void set_integral_limit(float limit);
        void set_output_limit(float limit);

        void set_filter(Filter *filter);
        void set_enable_derivative_filter(bool enable);
};

#endif // PID_CONTROLLER_H
