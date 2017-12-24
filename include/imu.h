#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "imutypes.h"

void init_mpu6050();
void read_angular_rates(int16_t gyro_axis[3]);
void read_abs_angles(axis_t *attitude);

#endif // IMU_H
