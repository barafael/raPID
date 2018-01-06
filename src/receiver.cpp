#include "Arduino.h"
#include "WProgram.h"

#include "../interface/receiver.h"

/* The servo interrupt writes to this variable and the receiver function reads */
static volatile channels_t receiver_in_shared;

/* Written by interrupt on rising edge */
static uint64_t receiver_pulse_start_time[NUM_CHANNELS] = { 0 };


/*
   —————————————————————————————————————————————————————————
   ———             RECEIVER READ FUNCTION                ———
   —————————————————————————————————————————————————————————
*/

/* Copy each new value */
void update_receiver(channels_t *receiver_in) {
    noInterrupts();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        receiver_in->channels[index] = receiver_in_shared.channels[index];
    }
    interrupts();

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        /* Indicate lost signal */
        if (receiver_in->channels[index] > 10000) receiver_in->channels[index] = 0;

        if (receiver_in->channels[index] < 1000) receiver_in->channels[index] = 1000;
        if (receiver_in->channels[index] > 2000) receiver_in->channels[index] = 2000;
    }
}

static bool has_signal() {
    noInterrupts();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        if (receiver_in_shared.channels[index] != 0) {
            interrupts();
            return true;
        }
    }
    interrupts();
    return false;
}

void update_throttle();
void update_roll();
void update_pitch();
void update_yaw();

bool init_receiver() {
    /* The pinMode should be set to input by default, set it anyway */
    pinMode(THROTTLE_INPUT_PIN, INPUT);
    pinMode(ROLL_INPUT_PIN,     INPUT);
    pinMode(PITCH_INPUT_PIN,    INPUT);
    pinMode(YAW_INPUT_PIN,      INPUT);

    /* On each CHANGE on an input pin, an interrupt handler is called */
    attachInterrupt(THROTTLE_INPUT_PIN, update_throttle, CHANGE);
    attachInterrupt(ROLL_INPUT_PIN,     update_roll,     CHANGE);
    attachInterrupt(PITCH_INPUT_PIN,    update_pitch,    CHANGE);
    attachInterrupt(YAW_INPUT_PIN,      update_yaw,      CHANGE);

    /*
    attachInterrupt(AUX1_INPUT_PIN,     update_aux1,      CHANGE);
    attachInterrupt(AUX2_INPUT_PIN,     update_aux2,      CHANGE);
    */

    delay(20);

    return has_signal();
}


/*
   ————————————————————————————————————————————————————————————————
   ———             RECEIVER READ INTERRUPT ROUTINES             ———
   ————————————————————————————————————————————————————————————————
*/

void update_throttle() {
    if (digitalRead(THROTTLE_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[THROTTLE_CHANNEL] = micros();
    } else {
        receiver_in_shared.channels[THROTTLE_CHANNEL] =
            (uint16_t)(micros() - receiver_pulse_start_time[THROTTLE_CHANNEL]);
    }
}

void update_roll() {
    if (digitalRead(ROLL_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[ROLL_CHANNEL] = micros();
    } else {
        receiver_in_shared.channels[ROLL_CHANNEL] = (uint16_t)(micros() - receiver_pulse_start_time[ROLL_CHANNEL]);
    }
}

void update_pitch() {
    if (digitalRead(PITCH_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[PITCH_CHANNEL] = micros();
    } else {
        receiver_in_shared.channels[PITCH_CHANNEL] = (uint16_t)(micros() - receiver_pulse_start_time[PITCH_CHANNEL]);
    }
}

void update_yaw() {
    if (digitalRead(YAW_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[YAW_CHANNEL] = micros();
    } else {
        receiver_in_shared.channels[YAW_CHANNEL] = (uint16_t)(micros() - receiver_pulse_start_time[YAW_CHANNEL]);
    }
}

void update_aux1() {
    if (digitalRead(AUX1_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[AUX1_CHANNEL] = micros();
    } else {
        receiver_in_shared.channels[AUX1_CHANNEL] = (uint16_t)(micros() - receiver_pulse_start_time[AUX1_CHANNEL]);
    }
}

void update_aux2() {
    if (digitalRead(AUX2_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[AUX2_CHANNEL] = micros();
    } else {
        receiver_in_shared.channels[AUX2_CHANNEL] = (uint16_t)(micros() - receiver_pulse_start_time[AUX2_CHANNEL]);
    }
}
