#ifndef PID_PARAM_H
#define PID_PARAM_H

typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;

    float integral_limit;
    float output_limit;
} pid_param_t;

pid_param_t pid_params_init(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit);

#endif // PID_PARAM_H
