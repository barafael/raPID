#include "../include/Mixer.h"
#include "../include/util.h"

Mixer::Mixer(float thr_vol, float roll_vol, float pitch_vol, float yaw_vol)
            : throttle_volume(thr_vol)
            , roll_volume    (roll_vol)
            , pitch_volume   (pitch_vol)
            , yaw_volume     (yaw_vol) {
    if (throttle_volume > 1.0 || throttle_volume < -1.0 ||
        roll_volume     > 1.0 || roll_volume     < -1.0 ||
        pitch_volume    > 1.0 || pitch_volume    < -1.0 ||
        yaw_volume      > 1.0 || yaw_volume      < -1.0) {

        //Serial.println(F("one of the volume parameters is out of range of [-1.0, 1.0]"));

        clamp(throttle_volume, -1.0, 1.0);
        clamp(roll_volume,     -1.0, 1.0);
        clamp(pitch_volume,    -1.0, 1.0);
        clamp(yaw_volume,      -1.0, 1.0);
    }
}
