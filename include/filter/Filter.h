#ifndef FILTER_H
#define FILTER_H

#include <cstddef>

template <typename T>
class Filter {
    public:
        virtual T next(T value) = 0;
};

#endif // FILTER_H

