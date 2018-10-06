#include "../../include/pid/PIDParams.h"

pid_params_t pid_params_init(float p_gain, float i_gain, float d_gain,
        float integral_limit, float output_limit) {
    pid_params_t params = {
    .p_gain = p_gain,
    .i_gain = i_gain,
    .d_gain = d_gain,
    .integral_limit = integral_limit,
    .output_limit   = output_limit
    };
    return params;
}

