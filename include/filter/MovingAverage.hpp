#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>
#include <vector>

#include "Filter.hpp"

class MovingAverage : public Filter {
    private:
        std::vector<float> values;
        size_t marker = 0;
        size_t n = 0;

    public:
        explicit MovingAverage(size_t n);
        float next(float value) override;
};

#endif // MOVING_AVERAGE_H
