#include "../../include/filter/moving_average_filter.h"

/*@ assigns \nothing;
*/
//ensures \valid(\result.values + (0 .. FILTER_SIZE));
// ensures \result.marker == 0;
moving_average_t init_moving_average_filter() {
    moving_average_t self;
    self.marker = 0;
    return self;
}

/*@ requires \valid(self);
    requires \is_finite(value);
    assigns self->marker \from value;
    assigns self->values[(self->marker + 1) % FILTER_SIZE];
*/
float moving_average_next(moving_average_t *self, float value) {
    //@ assert self->marker >= 0 && self->marker < FILTER_SIZE;
    self->marker = (self->marker + 1) % FILTER_SIZE;
    self->values[self->marker] = value;
    float sum = 0;
    for (size_t index = 0; index < FILTER_SIZE; index++) {
        sum += self->values[index];
    }
    return sum / (float) FILTER_SIZE;
}
