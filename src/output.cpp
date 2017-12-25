#include <stdint.h>

typedef struct {
    int8_t p_vol;
    int8_t i_vol;
    int8_t d_vol;
} volume_t;

typedef struct {
    int8_t throttle_vol;

    volume_t roll_vol;
    volume_t pitch_vol;
    volume_t yaw_vol;

    volume_t roll_autolevel_vol;
    volume_t pitch_autolevel_vol;
    volume_t yaw_autolevel_vol;
} mixer_t;

typedef struct {
    typedef enum { SERVO, ESC } type;
    uint8_t pin;
    uint16_t upper_limit = 2000;
    uint16_t lower_limit = 1000;
    typedef enum { STBL, RATE } mode;
    mixer_t mixer;
} output_t;

