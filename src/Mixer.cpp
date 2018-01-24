#include "../include/Mixer.h"

Mixer::Mixer(float thr_vol, float roll_vol, float pitch_vol, float yaw_vol)
            : throttle_volume { thr_vol }
            , roll_volume     { roll_vol }
            , pitch_volume    { pitch_vol }
            , yaw_volume      { yaw_vol } {}
