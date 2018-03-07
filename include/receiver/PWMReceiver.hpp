#ifndef PWMRECEIVER_H
#define PWMRECEIVER_H

#include "Receiver.hpp"
#include <vector>

class PWMReceiver : Receiver {
    private:
        /* TODO: maybe use static datastructure again */
        std::vector<uint8_t> pins;

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

        const void update(channels_t channels) override;

        void set_offsets(channels_t channels) override;
        void set_trims(channels_t channels) override;
        void set_inversion(inversion_t inversion) override;

        const bool has_signal() override;

        friend void update_throttle();
        friend void update_roll();
        friend void update_pitch();
        friend void update_yaw();
        friend void update_aux1();
        friend void update_aux2();
};

/* Since friend declarations are not forward declarations,
 * we have to repeat the signatures here in their actual scope
 */
void update_throttle();
void update_roll();
void update_pitch();
void update_yaw();
void update_aux1();
void update_aux2();

#endif // PWMRECEIVER_H
