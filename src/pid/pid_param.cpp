#include "../../include/pid/pid_param.h"

pid_param_t pid_params_init(float p_gain, float i_gain, float d_gain, float integral_limit, float output_limit) {
    pid_param_t param = {
        .p_gain         = p_gain,
        .i_gain         = i_gain,
        .d_gain         = d_gain,
        .integral_limit = integral_limit,
        .output_limit   = output_limit
    };
    return param;
}
