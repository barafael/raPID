#include "../include/PIDParams.h"

PIDParams::PIDParams(const float p_gain, const float i_gain, const float d_gain,
                     const float integral_limit, const float output_limit)
    : p_gain(p_gain)
    , i_gain(i_gain)
    , d_gain(d_gain)

    , integral_limit(integral_limit)
    , output_limit  (output_limit) {}

