#ifndef COMPLEMENTARY_FILTER
#define COMPLEMENTARY_FILTER

#include <stdint.h>

typedef struct {
    float prev_output;
    float beta;
    float ateb;
} complementary_filter_t;

complementary_filter_t init_complementary_filter(float beta);
float complementary_next(complementary_filter_t *self, float value);
void  complementary_set_beta(complementary_filter_t *self, float beta);

#endif // COMPLEMENTARY_FILTER
