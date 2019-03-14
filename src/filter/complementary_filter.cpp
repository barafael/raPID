#include "../../include/filter/complementary_filter.h"

/*@ requires (float)0 <= (float)number < (float)1.0;
    ensures (float)\result == (float)((float)1.0 - (float)number);
    ensures (float)((float)\result + (float)number) == (float)1.0;
    ensures (float)0 < (float)\result <= (float)1.0;
    ensures (float)\result > (float)0;
*/
float complement_float(float number) {
    return (float)1 - number;
}

/*@ requires \is_finite(beta);
    requires (float)0 <= beta < (float)1;
    ensures (float)\result.beta == (float)beta;
    ensures (float)0 <= (float)\result.ateb <= (float)1;
    ensures (float)((float)\result.beta + (float)\result.ateb) == (float)1;
*/
complementary_filter_t init_complementary_filter(float beta) {
    float ateb = (float)1 - beta;
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
