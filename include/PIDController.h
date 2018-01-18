#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* TODO implement explicit cascaded pid controller? */

#include <stdint.h>

class PIDController {
    public:
        float p_gain;
        float i_gain;
        float d_gain;

        PIDController(const float _p_gain, const float _i_gain, const float _d_gain,
                      const float _integral_limit, const float _output_limit);

        void set_enabled(bool enable);

        float compute(const uint64_t now, const float measured,
                const float setpoint);

        PIDController* set_p(const float _p_gain);
        PIDController* set_i(const float _i_gain);
        PIDController* set_d(const float _d_gain);

        PIDController* integral_reset();

    private:
        bool enabled = true;

        float integral;
        float integral_limit;

        float derivative;

        /* For derivative-on-error */
        float last_error;

        /* For derivative-on-setpoint */
        float last_setpoint;

        float output_limit;

        uint64_t last_time;
};

#endif // PID_CONTROLLER_H

