#include "../include/PIDParams.h"

PIDParams::PIDParams(const float _p_gain, const float _i_gain, const float _d_gain,
                     const float _integral_limit, const float _output_limit)
    : p_gain { _p_gain }
    , i_gain { _i_gain }
    , d_gain { _d_gain }

    , integral_limit { _integral_limit }

    , output_limit   { _output_limit } {}

