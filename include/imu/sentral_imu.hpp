#ifndef SENTRAL_IMU_H
#define SENTRAL_IMU_H

#include "stdint.h"

#include "../../include/pins.h"
#include "../axis.hpp"

typedef struct {
} sentral_imu_t;

sentral_imu_t init_sentral_imu();

vec3_t update_accelerometer(sentral_imu_t *self);
vec3_t update_gyroscope(sentral_imu_t *self);
vec3_t update_magnetometer(sentral_imu_t *self);

vec3_t update_attitude(sentral_imu_t *self);

#endif // SENTRAL_IMU_H
