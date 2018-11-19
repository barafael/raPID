#include "../../include/filter/moving_average_filter.h"

moving_average_t init_moving_average_filter() {
    moving_average_t self;
    self.marker = 0;
    return self;
}

float moving_average_next(moving_average_t *self, float value) {
    self->marker = (self->marker + 1) % FILTER_SIZE;
    self->values[self->marker] = value;
    float sum = 0;
    for (size_t index = 0; index < FILTER_SIZE; index++) {
        sum += self->values[index];
    }
    return sum / (float)FILTER_SIZE;
}