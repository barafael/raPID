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

/*@ axiomatic Sum_Int {
      logic int sum_int(int *values, integer begin, integer end) reads values[begin .. (end - 1)];

      axiom empty_int:
        \forall int *p, integer begin, end; begin >= end ==> sum_int(p, begin, end) == 0;

      axiom range_int:
        \forall int *p, integer begin, end; begin < end ==> sum_int(p, begin, end) == sum_int(p, begin, end - 1) + p[end - 1];
}
*/

/*@ requires \valid_read(values + (0 .. size - 1));

    assigns \nothing;

    ensures \result == sum_int(values, 0, size);
*/
int sum_int_array(const int *values, size_t size) {
    int sum = 0;
    /*@ loop invariant 0 <= index <= size;
        loop invariant sum == sum_int(values, 0, index);
        loop assigns sum, index;
        loop variant size - index;
    */
    for (size_t index = 0; index < size; index++) {
        sum += values[index];
    }
    return sum;
}

/*@ axiomatic Sum_Real {
      logic float sum_real(float *values, integer begin, integer end) reads values[begin .. (end - 1)];

      axiom empty_real:
        \forall float *p, integer begin, end; begin >= end ==> sum_real(p, begin, end) == 0;

      axiom range_real:
        \forall float *p, integer begin, end; begin < end ==> sum_real(p, begin, end) == sum_real(p, begin, end - 1) + p[end - 1];
}
*/

/*@ requires \valid_read(values + (0 .. size - 1));

    requires \forall integer i; 0 <= i < size ==> \is_finite(values[i]);
    assigns \nothing;

    ensures \result == sum_real(values, 0, size);
*/
float sum_float_array(const float *values, size_t size) {
    float sum = 0;
    /*@ loop invariant 0 <= index <= size;
        loop invariant sum == sum_real(values, 0, index);
        loop assigns sum, index;
        loop variant size - index;
    */
    for (size_t index = 0; index < size; index++) {
        sum += values[index];
    }
    return sum;
}

/*@ requires \valid(self);
    requires \valid(self->values + (0 .. FILTER_SIZE));
    requires \is_finite(value);

    assigns self->values[self->marker], self->marker;

    behavior increment:
        assumes self->marker < (FILTER_SIZE - 1);
        ensures self->values[(\old(self->marker))] == value;
        ensures \at(self->marker, Post) == 0;
    behavior reset:
        assumes self->marker == FILTER_SIZE - 1;
        ensures self->values[(\old(self->marker))] == value;
        ensures \at(self->marker, Post) == \old(self->marker) + 1;
    behavior never_happens:
        assumes self->marker >= FILTER_SIZE;
        ensures \false;

    disjoint behaviors increment, reset, never_happens;
    complete behaviors increment, reset, never_happens;
*/
float moving_average_next(moving_average_t *self, float value) {
    self->values[self->marker] = value;
    //@ assert \is_finite(self->values[self->marker]);
    self->marker = self->marker == (FILTER_SIZE - 1) ? 0 : self->marker + 1;
    //@ assert self->marker >= 0 && self->marker < FILTER_SIZE;
    float sum = sum_float_array(self->values, FILTER_SIZE);
    return sum / (float) FILTER_SIZE;
}
