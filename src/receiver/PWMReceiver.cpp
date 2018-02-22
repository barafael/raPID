#include "Arduino.h"

#include "../include/receiver/PWMReceiver.h"

/* Access variable for ISRs */
PWMReceiver *pwm_rx_instance = nullptr;

/* Array of void functions without params, one for each input */
static void (*interrupts[NUM_CHANNELS])() = {};

PWMReceiver::PWMReceiver(uint8_t throttle_pin, uint8_t roll_pin, uint8_t pitch_pin, uint8_t yaw_pin,
        uint8_t aux1_pin, uint8_t aux2_pin,
        channels_t offsets) {
    pwm_rx_instance = this;

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        this->offsets[index] = offsets[index];
    }

    interrupts[0] = update_throttle;
    interrupts[1] = update_roll;
    interrupts[2] = update_pitch;
    interrupts[3] = update_yaw;
    interrupts[4] = update_aux1;
    interrupts[5] = update_aux2;

    pins.reserve(6);

    pins[0] = throttle_pin;
    pins[1] = roll_pin;
    pins[2] = pitch_pin;
    pins[3] = yaw_pin;
    pins[4] = aux1_pin;
    pins[5] = aux2_pin;

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        attachInterrupt(pins[index], interrupts[index], CHANGE);
    }
}


/*
   ---------------------------------------------------------
   ---           PWMRECEIVER UPDATE FUNCTION             ---
   ---------------------------------------------------------
*/

const void PWMReceiver::update(channels_t channels) {
    noInterrupts();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        channels[index] = channels_shared[index];
    }
    interrupts();

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        clamp(channels[index], 1000, 2000);
        channels[index] += offsets[index];
        channels[index] += trims[index];
    }
}

void PWMReceiver::set_trims(channels_t trims) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        this->trims[index] = trims[index];
    }
}

const bool PWMReceiver::has_signal() {
    noInterrupts();
    for (size_t index = 0; index < 4; index++) {
        if (channels_shared[index] == 0) {
            interrupts();
            return false;
        }
    }
    interrupts();
    return true;
}


/*
   ----------------------------------------------------------------
   ---           PWMRECEIVER READ INTERRUPT ROUTINES            ---
   ----------------------------------------------------------------
*/

void update_throttle() {
    if (digitalRead(pwm_rx_instance->pins[0]) == HIGH) {
        pwm_rx_instance->pwm_pulse_start_time[THROTTLE_CHANNEL] = micros();
    } else {
        pwm_rx_instance->channels_shared[THROTTLE_CHANNEL] =
            (uint16_t)(micros() - pwm_rx_instance->pwm_pulse_start_time[THROTTLE_CHANNEL]);
    }
}

void update_roll() {
    if (digitalRead(pwm_rx_instance->pins[1]) == HIGH) {
        pwm_rx_instance->pwm_pulse_start_time[ROLL_CHANNEL] = micros();
    } else {
        pwm_rx_instance->channels_shared[ROLL_CHANNEL] =
            (uint16_t)(micros() - pwm_rx_instance->pwm_pulse_start_time[ROLL_CHANNEL]);
    }
}

void update_pitch() {
    if (digitalRead(pwm_rx_instance->pins[2]) == HIGH) {
        pwm_rx_instance->pwm_pulse_start_time[PITCH_CHANNEL] = micros();
    } else {
        pwm_rx_instance->channels_shared[PITCH_CHANNEL] =
            (uint16_t)(micros() - pwm_rx_instance->pwm_pulse_start_time[PITCH_CHANNEL]);
    }
}

void update_yaw() {
    if (digitalRead(pwm_rx_instance->pins[3]) == HIGH) {
        pwm_rx_instance->pwm_pulse_start_time[YAW_CHANNEL] = micros();
    } else {
        pwm_rx_instance->channels_shared[YAW_CHANNEL] =
            (uint16_t)(micros() - pwm_rx_instance->pwm_pulse_start_time[YAW_CHANNEL]);
    }
}

void update_aux1() {
    if (digitalRead(pwm_rx_instance->pins[4]) == HIGH) {
        pwm_rx_instance->pwm_pulse_start_time[AUX1_CHANNEL] = micros();
    } else {
        pwm_rx_instance->channels_shared[AUX1_CHANNEL] =
            (uint16_t)(micros() - pwm_rx_instance->pwm_pulse_start_time[AUX1_CHANNEL]);
    }
}

void update_aux2() {
    if (digitalRead(pwm_rx_instance->pins[5]) == HIGH) {
        pwm_rx_instance->pwm_pulse_start_time[AUX2_CHANNEL] = micros();
    } else {
        pwm_rx_instance->channels_shared[AUX2_CHANNEL] =
            (uint16_t)(micros() - pwm_rx_instance->pwm_pulse_start_time[AUX2_CHANNEL]);
    }
}
