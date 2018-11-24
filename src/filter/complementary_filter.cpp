#include "../../include/filter/complementary_filter.h"

/*@ requires \is_finite(beta);
    ensures 0.0 <= \result.beta <= 1.0;
    ensures 0.0 <= \result.ateb <= 1.0;
    ensures \result.beta + \result.ateb == 1.0;
*/
complementary_filter_t init_complementary_filter(float beta) {
    clamp(beta, 0.0f, 1.0f);
    float ateb = 1.0f - beta;
    complementary_filter_t self;
    self.beta = beta;
    self.ateb = ateb;
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
