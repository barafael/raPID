#include "../../include/imu/madgwick_filter.h"

#define DEFAULT_BETA 8.384266471f
#define DEFAULT_GYRO_ERROR 6.981317008f
#define DEFAULT_GYRO_DRIFT 0.0f

/*@ requires \valid(self);
    requires \is_finite(beta);
    ensures \valid(\old(self)) ==> \valid(self);
    ensures self->beta == beta;
*/
void set_beta(madgwick_filter_t *self, float beta) {
    self->beta = beta;
}

/*@ requires \valid(self);
    requires \is_finite(deltat);
    ensures \valid(\old(self)) ==> \valid(self);
    ensures self->deltat == deltat;
*/
void set_deltat(madgwick_filter_t *self, float deltat) {
    self->deltat = deltat;
}

/*@ requires \is_finite(deltat);
    ensures \result.beta == DEFAULT_BETA;
    ensures \result.deltat == deltat;
    ensures \result.gyro_meas_error == DEFAULT_GYRO_ERROR;
    ensures \result.gyro_meas_drift == DEFAULT_GYRO_DRIFT;
    ensures \result.quat.a == 1.0;
    ensures \result.quat.b == 0.0;
    ensures \result.quat.c == 0.0;
    ensures \result.quat.d == 0.0;
*/
madgwick_filter_t init_madgwick_filter(float deltat) {
    quaternion_t quat    = { 1.0, 0.0, 0.0, 0.0 };
    madgwick_filter_t self = {
        .beta            = DEFAULT_BETA,
        .deltat          = deltat,
        .gyro_meas_error = DEFAULT_GYRO_ERROR,
        .gyro_meas_drift = DEFAULT_GYRO_DRIFT,
        .quat            = quat
    };
    return self;
}

/*@ requires \valid(self);
    requires \is_finite(acc.x);
    requires \is_finite(acc.y);
    requires \is_finite(acc.z);
    requires \is_finite(gyro.x);
    requires \is_finite(gyro.y);
    requires \is_finite(gyro.z);
    requires \is_finite(mag.x);
    requires \is_finite(mag.y);
    requires \is_finite(mag.z);
    ensures \valid(\old(self)) ==> \valid(self);
*/
quaternion_t madgwick_update(madgwick_filter_t *self, vec3_t acc, vec3_t gyro, vec3_t mag) {
    float hx, hy, _2bx, _2bz;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1   = 2.0f * self->quat.a;
    float _2q2   = 2.0f * self->quat.b;
    float _2q3   = 2.0f * self->quat.c;
    float _2q4   = 2.0f * self->quat.d;
    float _2q1q3 = 2.0f * self->quat.a * self->quat.c;
    float _2q3q4 = 2.0f * self->quat.c * self->quat.d;
    float q1q1   = self->quat.a * self->quat.a;
    float q1q2   = self->quat.a * self->quat.b;
    float q1q3   = self->quat.a * self->quat.c;
    float q1q4   = self->quat.a * self->quat.d;
    float q2q2   = self->quat.b * self->quat.b;
    float q2q3   = self->quat.b * self->quat.c;
    float q2q4   = self->quat.b * self->quat.d;
    float q3q3   = self->quat.c * self->quat.c;
    float q3q4   = self->quat.c * self->quat.d;
    float q4q4   = self->quat.d * self->quat.d;

    // Normalise accelerometer measurement
    if (!normalize_vec3(&acc)) {
        return self->quat;
    }

    // Normalise magnetometer measurement
    if (!normalize_vec3(&mag)) {
        return self->quat;
    }

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * self->quat.a * mag.x;
    _2q1my = 2.0f * self->quat.a * mag.y;
    _2q1mz = 2.0f * self->quat.a * mag.z;
    _2q2mx = 2.0f * self->quat.b * mag.x;
    hx   = mag.x * q1q1 - _2q1my * self->quat.d + _2q1mz * self->quat.c + mag.x * q2q2 + _2q2 * mag.y * self->quat.c + _2q2 * mag.z * self->quat.d - mag.x * q3q3 - mag.x * q4q4;
    hy   = _2q1mx * self->quat.d + mag.y * q1q1 - _2q1mz * self->quat.b + _2q2mx * self->quat.c - mag.y * q2q2 + mag.y * q3q3 + _2q3 * mag.z * self->quat.d - mag.y * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * self->quat.c + _2q1my * self->quat.b + mag.z * q1q1 + _2q2mx * self->quat.d - mag.z * q2q2 + _2q3 * mag.y * self->quat.d - mag.z * q3q3 + mag.z * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    quaternion_t s;
    // Gradient decent algorithm corrective step
    s.a = -_2q3 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q2 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         _2bz * self->quat.c * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (-_2bx * self->quat.d + _2bz * self->quat.b) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         _2bx * self->quat.c * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s.b = _2q4 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q1 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         4.0f * self->quat.b * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc.z) +
         _2bz * self->quat.d * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (_2bx * self->quat.c + _2bz * self->quat.a) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         (_2bx * self->quat.d - _4bz * self->quat.b) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s.c = -_2q1 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q4 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         4.0f * self->quat.c * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc.z) +
         (-_4bx * self->quat.c - _2bz * self->quat.a) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (_2bx * self->quat.b + _2bz * self->quat.d) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         (_2bx * self->quat.a - _4bz * self->quat.c) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s.d = _2q2 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q3 * (2.0f * q1q2 + _2q3q4 - acc.y) +
         (-_4bx * self->quat.d + _2bz * self->quat.b) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (-_2bx * self->quat.a + _2bz * self->quat.c) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         _2bx * self->quat.b * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    // normalise step magnitude
    normalize_quat(&s);

    // Compute rate of change of quaternion
    quaternion_t qdot = differentiate_quat(self->beta, gyro, &(self->quat), &s);

    // Integrate to yield quaternion
    scalar_quat(self->deltat, &qdot);

    self->quat = add_quat(self->quat, qdot);

    normalize_quat(&(self->quat));

    return self->quat;
}
