#include "../include/axis.hpp"

bool normalize_vec3(vec3_t *vector) {
    float norm = sqrtf(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z);
    if (norm == 0.0f) {
        return false;
    }
    norm = 1.0f / norm;
    vector->x *= norm;
    vector->y *= norm;
    vector->z *= norm;
    return true;
}
