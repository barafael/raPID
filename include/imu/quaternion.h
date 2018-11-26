#ifndef QUATERNION_H
#define QUATERNION_H

#include "../axis.hpp"
#include <math.h>

#include "../ArduinoMock.h"

typedef struct {
    float a;
    float b;
    float c;
    float d;
} quaternion_t;

quaternion_t quat_from_array(float const q[4]);

void normalize_quat(quaternion_t *quat);

void scalar_quat(float f, quaternion_t *quat);

quaternion_t add_quat(quaternion_t q1, quaternion_t q2);

quaternion_t differentiate_quat(float beta, vec3_t gyro, quaternion_t *quat, quaternion_t *s);

vec3_t quat2euler(quaternion_t quat);

#endif // QUATERNION_H
