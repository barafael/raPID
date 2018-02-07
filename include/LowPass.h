#ifndef LOW_PASS
#define LOW_PASS

#include <stdint.h>

#include "Filter.h"

template <typename T>
class LowPass : Filter<T> {
    private:
        T prev_output = 0.0f;
        float beta = 1.0;
        float ateb = 0.0;

    public:
        explicit LowPass(float beta);
        T next(T value);
        void set_beta(float beta);
};

template <typename T>
T LowPass<T>::next(T value) {
    prev_output = prev_output * beta + value * ateb;
    return prev_output;
}

template <typename T>
void LowPass<T>::set_beta(float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        return;
    }
    this->beta = beta;
    this->ateb = 1.0 - beta;
}

#endif // LOW_PASS

