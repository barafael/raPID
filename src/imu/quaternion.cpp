#include "../../include/imu/quaternion.h"

/*@ requires \valid_read(q + (0 .. 3));
    ensures \result.a == q[0];
    ensures \result.b == q[1];
    ensures \result.c == q[2];
    ensures \result.d == q[3];
*/
quaternion_t quat_from_array(float const q[4]) {
    quaternion_t quat = { q[0], q[1], q[2], q[3] };
    return quat;
}

/*@ requires \valid(quat);
    requires \is_finite(quat->a);
    requires \is_finite(quat->b);
    requires \is_finite(quat->c);
    requires \is_finite(quat->d);

    ensures \valid(\old(quat)) ==> \valid(quat);

    assigns quat->a;
    assigns quat->b;
    assigns quat->c;
    assigns quat->d;

    behavior zero_quat:
      assumes \sqrt(quat->a * quat->a + quat->b * quat->b + quat->c * quat->c + quat->d * quat->d) == 0.0f;
      ensures \result == false;
      ensures quat->a == \old(quat->a);
      ensures quat->b == \old(quat->b);
      ensures quat->c == \old(quat->c);
      ensures quat->d == \old(quat->d);
    behavior valid_quat:
      assumes \sqrt(quat->a * quat->a + quat->b * quat->b + quat->c * quat->c + quat->d * quat->d) != 0.0f;
      assumes \sqrt(quat->a * quat->a + quat->b * quat->b + quat->c * quat->c + quat->d * quat->d) > 0.99f;
      assumes \sqrt(quat->a * quat->a + quat->b * quat->b + quat->c * quat->c + quat->d * quat->d) < 1.01f;
      ensures \result == true;

    disjoint behaviors zero_quat, valid_quat;
    complete behaviors zero_quat, valid_quat;
 */
void normalize_quat(quaternion_t *quat) {
    float norm = sqrtf(quat->a * quat->a + quat->b * quat->b + quat->c * quat->c + quat->d * quat->d);
    norm = 1.0f / norm;
    quat->a *= norm;
    quat->b *= norm;
    quat->c *= norm;
    quat->d *= norm;
}

/*@ requires \valid(quat);
    requires \is_finite(f);

    ensures \valid(\old(quat)) ==> \valid(quat);

    ensures quat->a == f * \old(quat->a);
    ensures quat->b == f * \old(quat->b);
    ensures quat->c == f * \old(quat->c);
    ensures quat->d == f * \old(quat->d);
*/
void scalar_quat(float f, quaternion_t *quat) {
    quat->a *= f;
    quat->b *= f;
    quat->c *= f;
    quat->d *= f;
}

/*@ ensures \result.a == q1.a + q2.a;
    ensures \result.b == q1.b + q2.b;
    ensures \result.c == q1.c + q2.c;
    ensures \result.d == q1.d + q2.d;
*/
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
    float pitch = atan2f(sinr, cosr);

    // pitch (y-axis rotation)
    float sinp = +2.0 * (quat.d * quat.b - quat.c * quat.a);
    float roll;
    if (fabsf(sinp) >= 1) {
        roll = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
        roll = asinf(sinp);
    }
    // yaw (z-axis rotation)
    float siny = +2.0 * (quat.d * quat.c + quat.a * quat.b);
    float cosy = +1.0 - 2.0 * (quat.b * quat.b + quat.c * quat.c);
    float yaw  = atan2f(siny, cosy);

    vec3_t euler = { roll, pitch, yaw };
    return euler;
}
