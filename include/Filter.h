#ifndef FILTER_H
#define FILTER_H

#include <cstddef>

template <typename T>
class Filter {
    virtual T next(T value);
};

#endif // FILTER_H

