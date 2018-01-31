#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* TODO implement explicit cascaded pid controller? */

#include <stdint.h>

#include "PIDParams.h"
#include "util.h"
#include "AVG_Filter.h"

typedef enum { ERROR, SETPOINT, MEASURED } derivative_type;

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

        static const size_t MAF_SIZE = 5;

        AVG_Filter<float, MAF_SIZE> deriv_filter;

    public:
        explicit PIDController(PIDParams *params);

        /* En/Disable Passthrough of setpoint */
        void set_enabled(bool enable);

        float compute(const uint64_t now, const float measured, const float setpoint);

        void set_p(const float _p_gain);
        void set_i(const float _i_gain);
        void set_d(const float _d_gain);

        void integral_reset();

        void set_derivative_type(derivative_type type);
        void enable_derivative_filter(bool enable);
};

#endif // PID_CONTROLLER_H

