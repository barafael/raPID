#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

class pid_result {
    public:
        float sum;
};

class pid_controller {
    public:
        float kp;
        float ki;
        float kd;

        pid_controller(const float kp, const float ki, const float kd,
                const float integral_limit, const float output_limit);

        pid_result compute(const uint64_t now, const float measured,
                const float setpoint);

        pid_controller* set_p(const float kp);
        pid_controller* set_i(const float ki);
        pid_controller* set_d(const float kd);

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

