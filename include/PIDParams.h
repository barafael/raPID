#ifndef PID_PARAMS_H
#define PID_PARAMS_H

template <typename T>
class PIDParams {
    public:
        T p_gain;
        T i_gain;
        T d_gain;

        T integral_limit;
        T output_limit;

        PIDParams<T>(const T _p_gain, const T _i_gain, const T _d_gain,
                  const T _integral_limit, const T _output_limit);
};

template<typename T>
PIDParams<T>::PIDParams(const T _p_gain, const T _i_gain, const T _d_gain,
                     const T _integral_limit, const T _output_limit)
    : p_gain { _p_gain }
    , i_gain { _i_gain }
    , d_gain { _d_gain }

    , integral_limit { _integral_limit }

    , output_limit   { _output_limit } {}

#endif //PID_PARAMS_H
