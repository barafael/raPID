#ifndef MADGWICK_IMU_H
#define MADGWICK_IMU_H

#include "../../include/imu/madgwick_filter.h"
#include "../axis.hpp"

typedef struct {
    madgwick_filter_t *filter;
} madgwick_imu_t;

madgwick_imu_t init_madgwick_imu();

vec3_t update_accelerometer(madgwick_imu_t *self);
vec3_t update_gyroscope(madgwick_imu_t *self);
vec3_t update_magnetometer(madgwick_imu_t *self);

vec3_t update_attitude(madgwick_imu_t *self);

#endif // MADGWICK_IMU_H
