#include "../../include/filter/complementary_filter.h"

/* axiomatic complement_real {
     logic real complement_real(float value);

     axiom compl:
       \forall real r; 0. <= r < 1.0 ==> 0.0 <= complement_real(r) < 1.0;
*/

/*@ requires 0.0 <= value < 1.0;
    ensures 0.0 < \result <= 1.0;
    ensures \result >= 1.0 - value - 0.01;
    ensures \result <= 1.0 - value + 0.01;
    ensures \result == 1.0 - value;
*/

float complement(float value) {
    return 1.0 - value;
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
