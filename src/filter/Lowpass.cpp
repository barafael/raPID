#include "../../include/filter/Lowpass.hpp"

Lowpass::Lowpass(float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        beta = 0.0f;
    }
    this->beta = beta;
    this->ateb = 1.0f - beta;
}

float Lowpass::next(float value) {
    prev_output = prev_output * beta + value * ateb;
    return prev_output;
}

void Lowpass::set_beta(float beta) {
    if (beta < 0.0f || beta > 1.0f) {
        return;
    }
    this->beta = beta;
    this->ateb = 1.0 - beta;
}

