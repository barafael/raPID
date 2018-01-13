#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* TODO implement explicit cascaded pid controller? */
#include <stdint.h>

class pid_controller {
    public:
        float p_coeff;
        float i_coeff;
        float d_coeff;

        pid_controller(const float p_coeff, const float i_coeff, const float d_coeff,
                const float integral_limit, const float output_limit);

        float compute(const uint64_t now, const float measured,
                const float setpoint);

        pid_controller* integral_reset();

        pid_controller* set_p(const float p_coeff);
        pid_controller* set_i(const float i_coeff);
        pid_controller* set_d(const float d_coeff);

    private:
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

