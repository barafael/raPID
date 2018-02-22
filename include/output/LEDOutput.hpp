#ifndef LED_OUTPUT
#define LED_OUTPUT

#include "Arduino.h"

#include "Output.hpp"

#include "util.h"
#include "settings.h"

class LEDOutput : Output {
    private:
        uint16_t brightness = 0;

        uint8_t resolution_bits = 16;
        float frequency_hz = 400.0f;
        float wavelength_sec = 1.0f/frequency_hz;

        // float min_pulse_width_sec = 0.001f;
        // float max_pulse_width_sec = 0.002f;

        uint8_t min_dutycycle_percent = 0;
        uint8_t max_dutycycle_percent = 80;

        uint16_t lower_limit = (1 << resolution_bits) * (min_dutycycle_percent / 100.0f);
        uint16_t upper_limit = (1 << resolution_bits) * (max_dutycycle_percent / 100.0f);

        uint16_t range = upper_limit - lower_limit;

        uint16_t throttle_low_cutoff = 25;

        bool low_throttle_cutoff_enabled = false;

        void write(uint16_t _milli_throttle);

    public:
        LEDOutput(const uint8_t pin,
                float throttle_volume,
                float roll_volume, float pitch_volume, float yaw_volume);

        void apply(uint16_t _milli_throttle,
                float roll_stbl, float pitch_stbl, float yaw_stbl) override;

        void set_resolution(uint8_t res);
        void set_frequency(uint32_t freq);

        void shut_off();

        void set_throttle_cutoff_enabled(bool enable);

        void set_limits(uint16_t lower, uint16_t upper);

        void set_throttle_volume(float volume) override;
        void set_roll_volume    (float volume) override;
        void set_pitch_volume   (float volume) override;
        void set_yaw_volume     (float volume) override;
};

#endif // FAST_PWM_OUTPUT

