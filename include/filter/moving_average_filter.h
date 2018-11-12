#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>
#include <stddef.h>

//TODO use dynamic size
const size_t FILTER_SIZE = 10;
typedef struct {
    float values[FILTER_SIZE];
    size_t marker;
} moving_average_t;

moving_average_t maf_next();
float filter_next(moving_average_t *self, float value);

#endif // MOVING_AVERAGE_H
