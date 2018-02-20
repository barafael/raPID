#ifndef RAW_IMU_H
#define RAW_IMU_H

#include <stdint.h>
#include <array>

#include "axis.h"

/* This class represents an interface to an absolutely unspectacular IMU sensor
 * that consists just of a gyroscope and accelerometer combo without and
 * internal motion processing. */

class RawIMU {
    private:
    public:
        virtual void update_gyroscope(axis_t& angular_rates) = 0;
        virtual void update_accelerometer(axis_t& acceleration) = 0;
        virtual void update_magnetometer(axis_t& magnetization) = 0;
};

#endif // RAW_IMU_H

