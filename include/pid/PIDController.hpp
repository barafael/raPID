#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* TODO implement explicit cascaded pid controller? */

#include <stdint.h>

#include "Arduino.h"

#include "PIDParams.hpp"
#include "../util.h"

#include "../filter/MovingAverage.hpp"
#include "../filter/Lowpass.hpp"

typedef enum { ERROR, SETPOINT, FEEDBACK } derivative_type;
typedef enum { NONE, MOVING_AVERAGE, LOWPASS } filter_type;

template<typename T>
class PIDController {
    private:
        bool enabled = true;

        T p_gain;
        T i_gain;
        T d_gain;

        T integral;
        T integral_limit;

        T derivative;

        derivative_type d_type = ERROR;

        /* For derivative-on-error */
        T last_error;

        /* For derivative-on-setpoint */
        T last_setpoint;

        /* For derivative-on-measured */
        T last_measured;

        T output_limit;

        uint64_t last_time;

        bool derivative_filter_enabled = false;
        filter_type derivative_filter_type = NONE;

        float lowpass_beta = 0.8;
        size_t mov_avg_size = 10;

        Lowpass<T> lowpass_filter  = Lowpass<T>(lowpass_beta);
        MovingAverage<T> ma_filter = MovingAverage<T>(mov_avg_size);

        Filter<T> *deriv_filter = nullptr;

    public:
        PIDController() = default;
        explicit PIDController(PIDParams<T> *params);
        PIDController(PIDParams<T> *params, float lowpass_beta);
        PIDController(PIDParams<T> *params, size_t mov_avg_size);

        /* En/Disable Passthrough of setpoint */
        void set_enabled(bool enable);

        T compute(const T measured, const T setpoint);

        void set_p(const T _p_gain);
        void set_i(const T _i_gain);
        void set_d(const T _d_gain);

        T get_p();
        T get_i();
        T get_d();

        void set_params(const PIDParams<T> *params);

        void integral_reset();

        void set_derivative_type(derivative_type type);
        void enable_derivative_filter(bool enable);
};

#include "../../src/pid/PIDController.tpp"

#endif // PID_CONTROLLER_H

