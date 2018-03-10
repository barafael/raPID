#include "../../include/filter/MovingAverage.hpp"

MovingAverage::MovingAverage(size_t n)
    : values(n)
    , n(n) {}

float MovingAverage::next(float value) {
    marker = (marker + 1) % n;
    values[marker] = value;
    float sum = 0;
    for (size_t index = 0; index < n; index++) {
        sum += values[index];
    }
    return sum / n;
}

