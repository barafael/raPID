#ifndef PWMRECEIVER_H
#define PWMRECEIVER_H

#include "../interface/Receiver.h"

class PWMReceiver : Receiver {
    private:
        uint8_t throttle_pin;
        uint8_t roll_pin;
        uint8_t pitch_pin;
        uint8_t yaw_pin;
        uint8_t aux1_pin;
        uint8_t aux2_pin;

        /* The servo interrupt writes to this variable and the receiver function reads */
        volatile channels_t receiver_in_shared = { 0 };

        /* Written by interrupt on rising edge */
        volatile channels_t receiver_pulse_start_time = { 0 };

    public:
        PWMReceiver(uint8_t _throttle_pin, uint8_t _roll_pin,
                    uint8_t _pitch_pin,    uint8_t _yaw_pin,
                    uint8_t _aux1_pin,     uint8_t _aux2_pin);

        const void update(channels_t channels);
        const bool has_signal();

        friend void update_throttle();
        friend void update_roll();
        friend void update_pitch();
        friend void update_yaw();
        friend void update_aux1();
        friend void update_aux2();
};

#endif // PWMRECEIVER_H
