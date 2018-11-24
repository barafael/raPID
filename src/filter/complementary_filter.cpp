#include "../../include/filter/complementary_filter.h"

/*@ requires \is_finite(beta);
    ensures \result.beta <= 1.0 && \result.beta >= 0;
    ensures \result.ateb <= 1.0 && \result.ateb >= 0;
    ensures \result.beta + \result.ateb == 1.0;
*/
complementary_filter_t init_complementary_filter(float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        beta = 1.0f;
    }
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
    assigns self->beta;
    assigns self->ateb;
    ensures self->beta <= 1.0 && self->beta >= 0.0;
    ensures self->ateb <= 1.0 && self->ateb >= 0.0;
    ensures self->beta  + self->ateb == 1.0;
*/
void complementary_filter_set_beta(complementary_filter_t *self, float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        return;
    }
    self->beta = beta;
    self->ateb = 1.0 - beta;
}
