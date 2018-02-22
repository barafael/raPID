#ifndef SOFTWARE_IMU_H
#define SOFTWARE_IMU_H

#include <stdint.h>
#include <array>

#include "RawIMU.hpp"

#include "axis.hpp"

class SoftwareIMU {
    private:
    protected:
        RawIMU *rawIMU = nullptr;
    public:
        explicit SoftwareIMU(RawIMU *rawIMU)
            : rawIMU(rawIMU) {}

        virtual void update_angular_rates(axis_t& angular_rates) = 0;
        virtual void update_attitude(axis_t& attitude) = 0;
};

#endif // SOFTWARE_IMU_H

