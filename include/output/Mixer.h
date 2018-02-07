#ifndef MIXER_H
#define MIXER_H

class Mixer {
    public:
        float throttle_volume;

        float roll_volume;
        float pitch_volume;
        float yaw_volume;

        Mixer(float thr_vol, float roll_vol, float pitch_vol, float yaw_vol);
};

#endif //MIXER_H
