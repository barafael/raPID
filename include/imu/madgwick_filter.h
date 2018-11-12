#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <math.h>

#include"../axis.hpp"
#include"quaternion.h"

#include<stdbool.h>

typedef struct {
    //float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
    float beta;

    /* integration interval */
    float deltat;

    // gyroscope measurement error in rads/s (start at 40 deg/s)
    // float GyroMeasError = M_PI * (40.0f / 180.0f);
    float gyro_meas_error;

    // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    //float GyroMeasDrift = M_PI * (0.0f  / 180.0f); // yes, really
    float gyro_meas_drift;

    quaternion_t quat;
} madgwick_filter_t;

madgwick_filter_t init_madgwick_filter(float deltat);

quaternion_t madgwick_update(madgwick_filter_t *self, vec3_t acc, vec3_t gyro, vec3_t mag);

void set_beta(madgwick_filter_t *self, float beta);
void set_deltat(madgwick_filter_t *self, float deltat);

#endif // MADGWICK_FILTER_H
