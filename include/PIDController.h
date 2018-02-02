#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* TODO implement explicit cascaded pid controller? */

#include <stdint.h>

#include "Arduino.h"

#include "PIDParams.h"
#include "util.h"
#include "MovingAverage.h"

typedef enum { ERROR, SETPOINT, MEASURED } derivative_type;

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

        static const size_t MAF_SIZE = 5;

        MovingAverage<T, MAF_SIZE> deriv_filter;

    public:
        PIDController() = default;
        explicit PIDController(PIDParams<T> *params);

        /* En/Disable Passthrough of setpoint */
        void set_enabled(bool enable);

        T compute(const T measured, const T setpoint);

        void set_p(const T _p_gain);
        void set_i(const T _i_gain);
        void set_d(const T _d_gain);

        void set_params(const PIDParams<T> *params);

        void integral_reset();

        void set_derivative_type(derivative_type type);
        void enable_derivative_filter(bool enable);
};

#include "../src/PIDController.tpp"

#endif // PID_CONTROLLER_H

