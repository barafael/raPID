#include "../../include/filter/complementary_filter.h"
#include <assert.h>

/* axiomatic complement_real {
     logic real complement_real(float value);

     axiom zero:
       \forall real r; r == 0.0f ==> complement_real(r) == 1.0f;
     axiom one:
       \forall real r; r == 1.0f ==> complement_real(r) == 0.0f;
     axiom compl:
       \forall real r; 0.0f < r < 1.0f ==> 0.0f <= complement_real(r) < 1.0f;
*/

/*@ requires 0.0 <= (float)number < (float)upper;
    ensures 0.0 < \result <= (float)upper;
    ensures \result >= (float)upper - (float)number - 0.01;
    ensures \result <= (float)upper - (float)number + 0.01;
    ensures \result >= 0.0f;
    ensures \result == (float)upper - (float)number;
*/
float complement_float(float upper, float number) {
    return (float)upper - (float)number;
}

/*//@ requires 0 <= number < upper;
    ensures \result + number == upper;
    ensures \result >= 0.0f;
    ensures \result <= upper;
*/
int complement_int(int upper, int number) {
    return upper - number;
}

/*@ requires \is_finite(beta);
    ensures 0.0 <= \result.beta <= 1.0;
    ensures 0.0 <= \result.ateb <= 1.0;
    ensures \result.beta + \result.ateb == 1.0;
*/
complementary_filter_t init_complementary_filter(float beta) {
    clamp(beta, 0.0f, 1.0f);
    //@ assert 0.0f <= beta <= 1.0f;
    float ateb = -beta;
    //@ assert ateb == -beta;
    //@ assert -1.0f <= ateb <= 0.0f;
    ateb += 1.0f;
    //@ assert 0.0f <= ateb <= 1.0f;
    //@ assert beta + ateb == 1.0f;
    ateb = 1.0f - beta;
    complementary_filter_t self = {
        .beta = beta,
        .ateb = ateb,
    };
    return self;
}

/*@ requires \valid(self);
    requires \is_finite(value);
    assigns self->prev_output;
    ensures self->prev_output == \old(self->prev_output) * self->beta + value * self->ateb;
*/
float complementary_next(complementary_filter_t *self, float value) {
    self->prev_output = self->prev_output * self->beta + value * self->ateb;
    return self->prev_output;
}

/*@ requires \valid(self);
    requires \is_finite(beta);
    assigns self->beta;
    assigns self->ateb;
    behavior valid_beta:
      assumes 0.0f <= beta <= 1.0f;
      ensures 0.0 <= self->beta <= 1.0;
      ensures 0.0 <= self->ateb <= 1.0;
      ensures self->ateb == 1.0 - beta;
      ensures self->beta + self->ateb == 1.0;
    behavior invalid_beta:
      assumes beta < 0.0f || beta > 1.0f;
      ensures \old(self->beta) == self->beta;
      ensures \old(self->ateb) == self->ateb;

    complete behaviors valid_beta, invalid_beta;
    disjoint behaviors valid_beta, invalid_beta;
*/
void complementary_filter_set_beta(complementary_filter_t *self, float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        return;
    }
    self->beta = beta;
    self->ateb = 1.0 - beta;
}

#define LOCAL_MAIN
#ifdef LOCAL_MAIN
/*@ requires value < 25; */
int local_main(int value) {
    float a = 0.45f;
    float complement = complement_float(1.0, a);
    assert(a + complement == 1.0f);

    int comp_int = complement_int(25, value);
    assert(value + comp_int == 25);

    int comp_int2 = complement_int(25, 10);
    assert(10 + comp_int2 == 25);
    return comp_int2;
}
#endif
