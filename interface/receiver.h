#ifndef RECEIVER_H
#define RECEIVER_H

#include "../interface/settings.h"
#include "../interface/pins.h"

class Receiver {
    /*
    typedef struct {
    uint16_t& operator[](size_t i) { return c[i]; }
    uint16_t c[NUM_CHANNELS] = { 0 };
    } channels_t;
    */
    private:
        uint8_t throttle_pin;
        uint8_t roll_pin;
        uint8_t pitch_pin;
        uint8_t yaw_pin;
        uint8_t aux1_pin;
        uint8_t aux2_pin;

        /* The servo interrupt writes to this variable and the receiver function reads */
        volatile uint16_t receiver_in_shared[NUM_CHANNELS] = { 0 };

        /* Written by interrupt on rising edge */
        volatile uint64_t receiver_pulse_start_time[NUM_CHANNELS] = { 0 };

    public:
        Receiver(uint8_t _throttle_pin, uint8_t _roll_pin,
                 uint8_t _pitch_pin,    uint8_t _yaw_pin,
                 uint8_t _aux1_pin,     uint8_t _aux2_pin);

        const void update(uint16_t channels[NUM_CHANNELS]);
        const bool has_signal();

        friend void update_throttle();
        friend void update_roll();
        friend void update_pitch();
        friend void update_yaw();
        friend void update_aux1();
        friend void update_aux2();
};

#endif // RECEIVER_H
