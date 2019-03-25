#ifndef SENTRAL_IMU_H
#define SENTRAL_IMU_H

#include <stdint.h>

#include "../Mock.h"
#include "../pins.h"
#include "../axis.hpp"

typedef enum {
    IMU_INIT_OK = 0,
    ROM_VERSION_ERROR,
    ADDRESS_MISMATCH,
    REVISION_ID_MISMATCH,
    EEPROM_CRC_ERROR,
    NO_EEPROM,
    CALIBRATION_FAILURE,
} IMU_INIT_ERROR;

IMU_INIT_ERROR init_sentral_imu();

vec3_t update_accelerometer();
vec3_t update_gyroscope();
vec3_t update_magnetometer();

vec3_t update_attitude();

#endif // SENTRAL_IMU_H
