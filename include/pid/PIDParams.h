#ifndef MISRAC_PID_PARAMS_H
#define MISRAC_PID_PARAMS_H

typedef struct {
        float p_gain;
        float i_gain;
        float d_gain;

        float integral_limit;
        float output_limit;

} pid_params_t;

pid_params_t pid_params_init(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit);

#endif // MISRAC_PID_PARAMS_H
