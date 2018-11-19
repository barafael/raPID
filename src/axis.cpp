#include "../include/axis.hpp"

/*@ requires \valid(vector);
    requires \is_finite(vector->x);
    requires \is_finite(vector->y);
    requires \is_finite(vector->z);
    behavior zero_vec:
      assumes \sqrt(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z) == 0.0f;
      ensures \result == false;
    behavior valid_vec:
      assumes \sqrt(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z) != 0.0f;
      ensures \sqrt(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z) > 0.99;
      ensures \sqrt(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z) < 1.01;
      ensures \result == true;
    disjoint behaviors zero_vec, valid_vec;
    complete behaviors zero_vec, valid_vec;
 */
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
