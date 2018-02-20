#ifndef SOFTWARE_IMU_H
#define SOFTWARE_IMU_H

#include <stdint.h>
#include <array>

#include "RawIMU.h"

#include "axis.h"

class SoftwareIMU {
    private:
    public:
        SoftwareIMU(RawIMU *rawIMU);
        virtual void update_angular_rates(axis_t& angular_rates) = 0;
        virtual void update_attitude(axis_t& attitude) = 0;
};

#endif // SOFTWARE_IMU_H

