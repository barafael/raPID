#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>
#include <vector>

#include "Filter.h"

template <typename T>
class MovingAverage : public Filter<T> {
    private:
        std::vector<T> values;
        size_t marker = 0;
        size_t n = 0;

    public:
        explicit MovingAverage(size_t n);
        T next(T value) override;
};

template <typename T>
MovingAverage<T>::MovingAverage(size_t n)
    : values(n)
    , n(n) {}

template <typename T>
T MovingAverage<T>::next(T value) {
    marker = (marker + 1) % n;
    values[marker] = value;
    T sum = 0;
    for (size_t index = 0; index < n; index++) {
        sum += values[index];
    }
    return sum / n;
}

#endif // MOVING_AVERAGE_H
