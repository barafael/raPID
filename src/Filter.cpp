#include "../include/Filter.h"

template <typename T, unsigned int n>
Filter<T, n>::Filter() {}

template <typename T, unsigned int n>
T Filter<T, n>::next(T const &value) {
    marker = (marker + 1) % n;
    values[marker] = value;
    T sum;
    for (int index = 0; index < n; index++) {
        sum += values[index];
    }
    return sum / n;
}

