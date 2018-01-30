#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <cstddef>

template <typename T, unsigned int n>
class Filter {
    private:
        T values[n];
        size_t marker = 0;

    public:
        Filter();
        T next(T const &);
};

#endif //FILTER_H
