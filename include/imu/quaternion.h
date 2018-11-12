#ifndef QUATERNION_H
#define QUATERNION_H

#include "../axis.hpp"
#include <math.h>

typedef struct {
    float a;
    float b;
    float c;
    float d;
} quaternion_t;

quaternion_t quat_from_array(float const q[4]) {
    quaternion_t quat = { q[0], q[1], q[2], q[3] };
    return quat;
}

void normalize_quat(quaternion_t *quat) {
    float norm = sqrtf(quat->a * quat->a + quat->b * quat->b + quat->c * quat->c + quat->d * quat->d);
    norm = 1.0f / norm;
    quat->a *= norm;
    quat->b *= norm;
    quat->c *= norm;
    quat->d *= norm;
}

void scalar_quat(float f, quaternion_t *quat) {
    quat->a *= f;
    quat->b *= f;
    quat->c *= f;
    quat->d *= f;
}

quaternion_t add_quat(quaternion_t q1, quaternion_t q2) {
    quaternion_t result;
    result.a = q1.a + q2.a;
    result.b = q1.b + q2.b;
    result.c = q1.c + q2.c;
    result.d = q1.d + q2.d;
    return result;
}

quaternion_t differentiate_quat(float beta, vec3_t gyro, quaternion_t *quat, quaternion_t *s) {
    quaternion_t dot;
    dot.a = 0.5f * (-quat->b * gyro.x - quat->c * gyro.y - quat->d * gyro.z) - beta * s->a;
    dot.b = 0.5f * (quat->a  * gyro.x + quat->c * gyro.z - quat->d * gyro.y) - beta * s->b;
    dot.c = 0.5f * (quat->a  * gyro.y - quat->b * gyro.z + quat->d * gyro.x) - beta * s->c;
    dot.d = 0.5f * (quat->a  * gyro.z + quat->b * gyro.y - quat->c * gyro.x) - beta * s->d;
    return dot;
}

vec3_t quat2euler(const quaternion_t quat) {
    // roll (x-axis rotation)
    float sinr  = +2.0 * (quat.d * quat.a + quat.b * quat.c);
    float cosr  = +1.0 - 2.0 * (quat.a * quat.a + quat.b * quat.b);
    float pitch = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    float sinp = +2.0 * (quat.d * quat.b - quat.c * quat.a);
    float roll;
    if (fabs(sinp) >= 1) {
        roll = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
        roll = asin(sinp);
    }
    // yaw (z-axis rotation)
    float siny = +2.0 * (quat.d * quat.c + quat.a * quat.b);
    float cosy = +1.0 - 2.0 * (quat.b * quat.b + quat.c * quat.c);
    float yaw  = atan2(siny, cosy);

    vec3_t euler = { roll, pitch, yaw };
    return euler;
}

#endif // QUATERNION_H
