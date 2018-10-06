#ifndef PID_PARAMS_H
#define PID_PARAMS_H

class PIDParams {
    public:
        float p_gain;
        float i_gain;
        float d_gain;

        float integral_limit;
        float output_limit;

        PIDParams(float p_gain, float i_gain, float d_gain,
                  float integral_limit, float output_limit);
};

#endif //PID_PARAMS_H
