#include "../interface/output_mixer.h"

Output_mixer::Output_mixer(void) {
    this->type = ESC;
    this->pin = 22;
    mixer_t mixer;
    mixer.throttle_vol = 100;
    mixer.rate = { 100, 0, 0 };
    mixer.stbl = { 100, 0, 0 };
    this->mixer = mixer;
}

