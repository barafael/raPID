#include "../../include/filter/complementary_filter.h"

complementary_filter_t init_complementary_filter(float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        beta = 1.0f;
    }
    float ateb = 1.0 - beta;
    complementary_filter_t self;
    self.beta = beta;
    self.ateb = ateb;
    return self;
}

float complementary_next(complementary_filter_t *self, float value) {
    self->prev_output = self->prev_output * self->beta + value * self->ateb;
    return self->prev_output;
}

void complementary_filter_set_beta(complementary_filter_t *self, float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        return;
    }
    self->beta = beta;
    self->ateb = 1.0 - beta;
}
