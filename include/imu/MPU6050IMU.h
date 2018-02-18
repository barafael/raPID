#ifndef MPU6050IMU_H
#define MPU6050IMU_H

#include <stdint.h>
#include <array>

#include "Arduino.h"

#include "IMU.h"

#include "../include/error_blink.h"
#include "../include/pins.h"
#include "../include/settings.h"

static const uint16_t MPU6050_ACCEL_OFFSET_X = 1030;
static const uint16_t MPU6050_ACCEL_OFFSET_Y = -758;
static const uint16_t MPU6050_ACCEL_OFFSET_Z = 1257;
static const uint16_t MPU6050_GYRO_OFFSET_X  = 76;
static const uint16_t MPU6050_GYRO_OFFSET_Y  = -57;
static const uint16_t MPU6050_GYRO_OFFSET_Z  = -65;

class MPU6050IMU : IMU {
    private:
    public:
        MPU6050IMU();
        void update_angular_rates(axis_t& angular_rates) override;
        void update_attitude(axis_t& attitude) override;
};

#endif // MPU6050IMU_H

