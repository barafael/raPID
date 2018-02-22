#ifndef HARDWARE_IMU_H
#define HARDWARE_IMU_H

#include <stdint.h>
#include <array>

#include "axis.hpp"

class HardwareIMU {
    private:
    public:
        virtual void update_angular_rates(axis_t& angular_rates) = 0;
        virtual void update_attitude(axis_t& attitude) = 0;
};

#endif // HARDWARE_IMU_H

