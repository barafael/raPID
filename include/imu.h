#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#include "Vector3.h"

using axis = enum {
    ROLL_AXIS  = 0,
    PITCH_AXIS = 1,
    YAW_AXIS   = 2,
};

using axis_t = int16_t[3];
//using axis_t = Vector3<int16_t>;
using offset_axis_t = int64_t[3];

void init_mpu6050();
void update_angular_rates(axis_t angular_rates);
void update_attitude(axis_t attitude);

#endif // IMU_H

