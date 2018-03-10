#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

#include "Arduino.h"

#include "PIDParams.hpp"
#include "../util.h"

#include "../filter/MovingAverage.hpp"
#include "../filter/Lowpass.hpp"

typedef enum { ERROR, SETPOINT, FEEDBACK } derivative_type;
typedef enum { NONE, MOVING_AVERAGE, LOWPASS } filter_type;

class PIDController {
    private:
        bool enabled = true;

        float p_gain;
        float i_gain;
        float d_gain;

        float integral;
        float integral_limit;

        float derivative;

        derivative_type d_type = ERROR;

        /* For derivative-on-error */
        float last_error;

        /* For derivative-on-setpoint */
        float last_setpoint;

        /* For derivative-on-measured */
        float last_measured;

        float output_limit;

        uint64_t last_time;

        bool derivative_filter_enabled = false;
        filter_type derivative_filter_type = NONE;

        float lowpass_beta = 0.8;
        size_t mov_avg_size = 10;

        Lowpass lowpass_filter  = Lowpass(lowpass_beta);
        MovingAverage ma_filter = MovingAverage(mov_avg_size);

        Filter *deriv_filter = nullptr;

    public:
        PIDController() = default;
        explicit PIDController(PIDParams& params);
        PIDController(PIDParams& params, float lowpass_beta);
        PIDController(PIDParams& params, size_t mov_avg_size);

        /* En/Disable Passthrough of setpoint */
        void set_enabled(bool enable);

        float compute(const float measured, const float setpoint);

        void set_p(const float _p_gain);
        void set_i(const float _i_gain);
        void set_d(const float _d_gain);

        void set_params(const PIDParams& params);

        float get_p();
        float get_i();
        float get_d();

        void integral_reset();

        void set_derivative_type(derivative_type type);
        void enable_derivative_filter(bool enable);
};

#endif // PID_CONTROLLER_H
