#ifndef LOW_PASS
#define LOW_PASS

#include <stdint.h>

#include "Filter.hpp"

template <typename T>
class Lowpass : public Filter<T> {
    private:
        T prev_output = 0;
        float beta = 1.0;
        float ateb = 0.0;

    public:
        explicit Lowpass(float beta);
        T next(T value) override;
        void set_beta(float beta);
};

template <typename T>
Lowpass<T>::Lowpass(float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        beta = 0.0f;
    }
    this->beta = beta;
    this->ateb = 1.0f - beta;
}

template <typename T>
T Lowpass<T>::next(T value) {
    prev_output = prev_output * beta + value * ateb;
    return prev_output;
}

template <typename T>
void Lowpass<T>::set_beta(float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        return;
    }
    this->beta = beta;
    this->ateb = 1.0 - beta;
}

#endif // LOW_PASS

