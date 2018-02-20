#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include <math.h>

#include "SoftwareIMU.h"

#include "axis.hpp"

class MahonyFilter : public SoftwareIMU {
    private:
        float Kp;
        float Ki;

        float deltat = 0.0f;
        float sum = 0.0f;

        float integral_error[3] = {0.0f, 0.0f, 0.0f};

        axis_t angular_rates;
        axis_t acceleration;
        axis_t magnetization;

    public:
        MahonyFilter(RawIMU *rawIMU, float Kp = 10.0f, float Ki = 0.0f);

        void update(float q[4]);
};

#endif // MAHONY_FILTER_H
