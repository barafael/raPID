#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <array>

using axis = enum {
    ROLL_AXIS  = 0,
    PITCH_AXIS = 1,
    YAW_AXIS   = 2,
};

using axis_t = std::array<int16_t, 3>;

class IMU {
    private:
    public:
        virtual void update_angular_rates(axis_t& angular_rates) = 0;
        virtual void update_attitude(axis_t& attitude) = 0;
};

#endif // IMU_H
