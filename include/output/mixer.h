#ifndef MISRAC_MIXER_H
#define MISRAC_MIXER_H

#include <stdint.h>

typedef struct {
    float throttle_volume;

    float roll_volume;
    float pitch_volume;
    float yaw_volume;
} mixer_t;

mixer_t mixer_init(float thr_vol, float roll_vol, float pitch_vol, float yaw_vol);

#endif //MISRAC_MIXER_H
