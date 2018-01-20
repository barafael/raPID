#ifndef PID_PARAMS_H
#define PID_PARAMS_H

class PIDParams {
    public:
        float p_gain;
        float i_gain;
        float d_gain;

        float integral_limit;
        float output_limit;

        PIDParams(const float _p_gain, const float _i_gain, const float _d_gain,
                  const float _integral_limit, const float _output_limit);
};

#endif //PID_PARAMS_H
