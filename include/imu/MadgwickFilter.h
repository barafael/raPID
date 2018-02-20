#include <math.h>

#include "SoftwareIMU.h"

#include "axis.h"

class MadgwickFilter : public SoftwareIMU {
    private:
        float beta = 0.0f;
        float zeta = 0.0f;

        /* integration interval */
        float deltat = 0.0f;

        float sum = 0.0f;

        axis_t angular_rates;
        axis_t acceleration;
        axis_t magnetization;

    public:
        MadgwickFilter(RawIMU *rawIMU, float gyroMeasErr, float gyroMeasDrift);

        void update(float q[4]);
};
