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

        PIDParams<T>(const T p_gain, const T i_gain, const T d_gain,
                  const T integral_limit, const T output_limit);
};

template<typename T>
PIDParams<T>::PIDParams(const T p_gain, const T i_gain, const T d_gain,
                     const T integral_limit, const T output_limit)
    : p_gain(p_gain)
    , i_gain(i_gain)
    , d_gain(d_gain)

    , integral_limit(integral_limit)

    , output_limit  (output_limit) {}

#endif //PID_PARAMS_H
