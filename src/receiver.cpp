#include "Arduino.h"
#include "WProgram.h"

#include "../interface/receiver.h"

/* Access variable for ISRs */
Receiver *instance = NULL;

/*
   ————————————————————————————————————————————————————————————————
   ———             RECEIVER READ INTERRUPT ROUTINES             ———
   ————————————————————————————————————————————————————————————————
*/

void update_throttle() {
    if (digitalRead(instance->throttle_pin) == HIGH) {
        instance->receiver_pulse_start_time[THROTTLE_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[THROTTLE_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[THROTTLE_CHANNEL]);
    }
}

void update_roll() {
    if (digitalRead(instance->roll_pin) == HIGH) {
        instance->receiver_pulse_start_time[ROLL_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[ROLL_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[ROLL_CHANNEL]);
    }
}

void update_pitch() {
    if (digitalRead(instance->pitch_pin) == HIGH) {
        instance->receiver_pulse_start_time[PITCH_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[PITCH_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[PITCH_CHANNEL]);
    }
}

void update_yaw() {
    if (digitalRead(instance->yaw_pin) == HIGH) {
        instance->receiver_pulse_start_time[YAW_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[YAW_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[YAW_CHANNEL]);
    }
}

void update_aux1() {
    if (digitalRead(instance->aux1_pin) == HIGH) {
        instance->receiver_pulse_start_time[AUX1_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[AUX1_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[AUX1_CHANNEL]);
    }
}

void update_aux2() {
    if (digitalRead(instance->aux2_pin) == HIGH) {
        instance->receiver_pulse_start_time[AUX2_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[AUX2_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[AUX2_CHANNEL]);
    }
}

Receiver::Receiver(uint8_t _throttle_pin, uint8_t _roll_pin,
                   uint8_t _pitch_pin,    uint8_t _yaw_pin,
                   uint8_t _aux1_pin,     uint8_t _aux2_pin) {

    instance = this;

    throttle_pin = _throttle_pin;
    roll_pin     = _roll_pin;
    pitch_pin    = _pitch_pin;
    yaw_pin      = _yaw_pin;
    aux1_pin     = _aux1_pin;
    aux2_pin     = _aux2_pin;

    /* The pinMode should be set to input by default, set it anyway */
    pinMode(throttle_pin, INPUT);
    pinMode(roll_pin,     INPUT);
    pinMode(pitch_pin,    INPUT);
    pinMode(yaw_pin,      INPUT);
    pinMode(aux1_pin,     INPUT);
    pinMode(aux2_pin,     INPUT);

    /* On each CHANGE on an input pin, an interrupt handler is called */
    attachInterrupt(throttle_pin, update_throttle, CHANGE);
    attachInterrupt(roll_pin,     update_roll,     CHANGE);
    attachInterrupt(pitch_pin,    update_pitch,    CHANGE);
    attachInterrupt(yaw_pin,      update_yaw,      CHANGE);
    attachInterrupt(aux1_pin,     update_aux1,     CHANGE);
    attachInterrupt(aux2_pin,     update_aux2,     CHANGE);

    /* TODO Check if delay necessary for has_signal */
    delay(10);
}


/*
   —————————————————————————————————————————————————————————
   ———             RECEIVER UPDATE FUNCTION              ———
   —————————————————————————————————————————————————————————
*/

const void Receiver::update(channels_t channels) {
    noInterrupts();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        channels[index] = receiver_in_shared[index];
    }
    interrupts();

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        if (channels[index] < 1000) channels[index] = 1000;
        if (channels[index] > 2000) channels[index] = 2000;
        channels[index] -= 1000;
    }
}

const bool Receiver::has_signal() {
    noInterrupts();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        if (receiver_in_shared[index] == 0) {
            interrupts();
            return false;
        }
    }
    interrupts();
    return true;
}

