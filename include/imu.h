#ifndef IMU_H
#define IMU_H

#include <stdint.h>

class axis_t {
    public:
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
};

void init_mpu6050();
void update_angular_rates(axis_t *angular_rates);
void update_abs_angles(axis_t *attitude);

#endif // IMU_H

