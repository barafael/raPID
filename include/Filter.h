#ifndef FILTER_H
#define FILTER_H

#include <cstddef>

template <typename T, std::size_t n>
class Filter {
    virtual T next(T value);
};

#endif // FILTER_H

