#ifndef RECEIVER_H
#define RECEIVER_H

#include "settings.h"
#include "pins.h"

class Receiver {
    private:
        uint8_t throttle_pin;
        uint8_t aileron_pin;
        uint8_t elevator_pin;
        uint8_t rudder_pin;
        uint8_t aux1_pin;
        uint8_t aux2_pin;

        /* The servo interrupt writes to this variable and the receiver function reads */
        volatile uint16_t receiver_in_shared[NUM_CHANNELS] = { 0 };

        volatile uint16_t receiver_in[NUM_CHANNELS] = { 0 };

        /* Written by interrupt on rising edge */
        volatile uint64_t receiver_pulse_start_time[NUM_CHANNELS] = { 0 };

    public:
        Receiver(uint8_t _throttle_pin, uint8_t _aileron_pin,
                 uint8_t _elevator_pin, uint8_t _rudder_pin,
                 uint8_t _aux1_pin,     uint8_t _aux2_pin);

        void update();
        void update(uint16_t channels[NUM_CHANNELS]);
        bool has_signal();
        void get_channels(uint16_t channels[NUM_CHANNELS]);

        uint16_t get_throttle();
        uint16_t get_aileron();
        uint16_t get_elevator();
        uint16_t get_rudder();
        uint16_t get_aux1();
        uint16_t get_aux2();

        friend void update_throttle();
        friend void update_aileron();
        friend void update_elevator();
        friend void update_rudder();
        friend void update_aux1();
        friend void update_aux2();
};

#endif // RECEIVER_H
