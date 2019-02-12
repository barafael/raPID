#include "../../include/filter/moving_average_filter.h"

/*@ requires MAFinitialization: ghost_maf_status == MAF_UNINITIALIZED;
    assigns MAFinitialization: ghost_maf_status;
    ensures MAFvalidMarker: \result.marker == 0;
    ensures MAFinitialization: ghost_maf_status == MAF_INITIALIZED;
*/
moving_average_t init_moving_average_filter() {
    moving_average_t self;
    self.marker = 0;
    //@ ghost ghost_maf_status = MAF_INITIALIZED;
    return self;
}

/*@ requires size >= 0 && \valid(values + (0 .. size - 1));
    ensures \result == \sum(0, size - 1, \lambda integer i; values[i]); */
float array_sum_lambda_float(float values[], int size) {
    int index;
    float sum = 0.0;
    /*@ loop invariant 0 <= index <= size;
        loop invariant sum == \sum(0, index - 1, \lambda integer i; values[i]);
        loop variant size - index; */
    for (index = 0; index < size; index++) {
        sum += values[index];
    }
    return sum;
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

    ensures MAFaverage: result_is_sum: \result == sum_int(values, 0, size);
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

    ensures MAFaverage: \result == sum_real(values, 0, size);
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

/*@ requires valid_access: \valid(self);
    requires valid_access: \valid(self->values + (0 .. FILTER_SIZE));
    requires \is_finite(value);
    requires is_initialized: ghost_maf_status == MAF_INITIALIZED;
    requires MAFoutOfBounds: 0 <= self->marker < FILTER_SIZE;

    assigns self->values[self->marker], self->marker;

    behavior increment:
        assumes MAFoutOfBounds: 0 <= self->marker < (FILTER_SIZE - 1);
        ensures MAFoutOfBounds: self->marker == \old(self->marker) + 1;
        ensures self->values[(\old(self->marker))] == value;
        ensures MAFaverage: \result == sum_real(self->values, 0, FILTER_SIZE) / FILTER_SIZE;
    behavior reset:
        assumes MAFoutOfBounds: self->marker == FILTER_SIZE - 1;
        ensures MAFoutOfBounds: self->marker == 0;
        ensures self->values[(\old(self->marker))] == value;
        ensures \at(self->marker, Post) == \old(self->marker) + 1;
        ensures  MAFaverage: \result == sum_real(self->values, 0, FILTER_SIZE) / FILTER_SIZE;
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
    //@ assert MAFoutOfBounds: 0 <= self->marker < FILTER_SIZE;
    float sum = sum_float_array(self->values, FILTER_SIZE);
    return sum / (float) FILTER_SIZE;
}
