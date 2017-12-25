#ifndef OUTPUT_MIXER_H
#define OUTPUT_MIXER_H

#include <stdint.h>

typedef struct {
    int8_t r_vol;
    int8_t p_vol;
    int8_t y_vol;
} rpy_volume_t;

typedef struct {
    uint8_t throttle_vol;

    rpy_volume_t rate;
    rpy_volume_t stbl;
} mixer_t;

typedef enum { SERVO, ESC } type_t;
typedef enum { STBL, RATE } mode_t;

class Output_mixer {
    public:
        type_t type;
        uint8_t pin;
        mixer_t mixer;
        uint16_t upper_limit = 2000;
        uint16_t lower_limit = 1000;

        Output_mixer();
};

#endif // OUTPUT_MIXER_H
