#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>

#include "Filter.h"

template <typename T, std::size_t n>
class MovingAverage : Filter<T, n> {
    private:
        T values[n] = {};
        size_t marker = 0;

    public:
        T next(T value);
};

template <typename T, std::size_t n>
T MovingAverage<T, n>::next(T value) {
    marker = (marker + 1) % n;
    values[marker] = value;
    T sum = 0;
    for (size_t index = 0; index < n; index++) {
        sum += values[index];
    }
    return sum / n;
}

#endif // MOVING_AVERAGE_H

