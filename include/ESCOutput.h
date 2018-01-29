#ifndef ESC_OUTPUT_H
#define ESC_OUTPUT_H

#include "Arduino.h"

#include "Output.h"

#include "Servo.h"

#include "./util.h"

/* TODO: use this and generate pwm from analog pins
 * (Depends on new board, 3.5 port)
 */
typedef enum { PWM_STANDARD, PWM_FAST } pwm_frequency;

static const uint16_t THROTTLE_LOW_CUTOFF = 25;
static const uint16_t BASE_PULSE_MS = 800;

/* TODO: eventually, this class should refrain from using Servo.h
 * The servo library is limited to 50Hz and standard servo waveform.
 */
class ESCOutput : Output {
    private:
        Servo output;

        uint16_t milli_throttle = 0;

        uint16_t upper_limit = 1000;
        uint16_t lower_limit = 0;
        uint16_t range = upper_limit - lower_limit;

        void write(uint16_t _milli_throttle);

    public:
        ESCOutput(const uint8_t pin,
                float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume)
            : Output(pin, throttle_volume, roll_volume, pitch_volume, yaw_volume) {
                output.attach(pin);
            }

        void apply(uint16_t throttle,
                float roll_stbl, float pitch_stbl, float yaw_stbl);

        void shut_off();

        void set_limits(uint16_t lower, uint16_t upper);

        void set_throttle_volume(float volume);

        void set_roll_volume (float volume);
        void set_pitch_volume(float volume);
        void set_yaw_volume  (float volume);
};

#endif // ESC_OUTPUT_H

