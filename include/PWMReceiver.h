#ifndef PWMRECEIVER_H
#define PWMRECEIVER_H

#include "Receiver.h"
#include <vector>

class PWMReceiver : Receiver {
    private:
        /* TODO: maybe use static datastructure again */
        std::vector<uint8_t> pins;

        /* Channel offsets and throttle zero-point */
        channels_t offsets = { 0 };

        /* Per-channel trim */
        channels_t trims = { 0 };

        /* Interrupts write to this array and the update function reads
         * Note: disable interrupts when reading to avoid race conditions
         */
        volatile channels_t channels_shared = { 0 };

        /* Written by interrupt on rising edge, read on falling edge
         * No synchronization necessary if an interrupt only touches one array member.
         */
        volatile channels_t pwm_pulse_start_time = { 0 };

    public:
        PWMReceiver(uint8_t throttle_pin, uint8_t roll_pin, uint8_t pitch_pin, uint8_t yaw_pin,
                             uint8_t aux1_pin, uint8_t aux2_pin,
                             channels_t offsets);

        const void update(channels_t channels);

        void set_trims(channels_t channels);

        const bool has_signal();

        friend void update_throttle();
        friend void update_roll();
        friend void update_pitch();
        friend void update_yaw();
        friend void update_aux1();
        friend void update_aux2();
};

void update_throttle();
void update_roll();
void update_pitch();
void update_yaw();
void update_aux1();
void update_aux2();

#endif // PWMRECEIVER_H
