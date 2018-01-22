#ifndef MIXER_H
#define MIXER_H

class Mixer {
    public:
        float throttle_volume;

        float roll_volume;
        float pitch_volume;
        float yaw_volume;

        Mixer(float thr_vol, float roll_vol, float pitch_vol, float yaw_vol)
            : throttle_volume { thr_vol }
            , roll_volume     { roll_vol }
            , pitch_volume    { pitch_vol }
            , yaw_volume      { yaw_vol } {}
};

#endif //MIXER_H
