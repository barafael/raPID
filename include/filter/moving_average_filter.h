#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>
#include <stddef.h>

//TODO use dynamic size
#define FILTER_SIZE 10

typedef struct {
    float values[FILTER_SIZE];
    size_t marker;
} moving_average_t;

moving_average_t init_moving_average_filter();
float moving_average_next(moving_average_t *self, float value);

#endif // MOVING_AVERAGE_H
