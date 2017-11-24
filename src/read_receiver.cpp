#include <Arduino.h>
#include "../teensy3/WProgram.h"

#include "../include/read_receiver.h"

extern uint16_t receiver_in[NUM_CHANNELS];

static volatile uint8_t input_flags;

/* The servo interrupt writes to this variable and the receiver function reads */
static volatile uint16_t receiver_in_shared[NUM_CHANNELS] = { 0 };

/* Written by interrupt on rising edge */
static uint64_t receiver_pulse_start_time[NUM_CHANNELS] = { 0 };


/*
   —————————————————————————————————————————————————————————
   ———             RECEIVER READ FUNCTION                ———
   —————————————————————————————————————————————————————————
*/

/* Read each new value, indicated by the corresponding bit set in input_flags */
void read_receiver() {
    noInterrupts();
    for (size_t channel = 0; channel < NUM_CHANNELS; channel++) {
            receiver_in[channel] = receiver_in_shared[channel];
    }
    interrupts();

    for (size_t channel = 0; channel < NUM_CHANNELS; channel++) {
        receiver_in[channel] =
                receiver_in[channel] < 1000 ? 1000 : (receiver_in[channel]);
        receiver_in[channel] =
                receiver_in[channel] > 2000 ? 2000 : (receiver_in[channel]);
    }
}


static bool has_signal_on_init() {
    noInterrupts();
    for (size_t channel = 0; channel < NUM_CHANNELS; channel++) {
        if (receiver_in_shared[channel] != 0) {
            interrupts();
            return true;
        }
    }
    interrupts();
    return false;
}

void read_throttle();
void read_roll();
void read_pitch();
void read_yaw();

void init_rx_interrupts() {
    /* The pinMode should be set to input by default, set it anyway */
    pinMode(THROTTLE_INPUT_PIN, INPUT);
    pinMode(ROLL_INPUT_PIN,     INPUT);
    pinMode(PITCH_INPUT_PIN,    INPUT);
    pinMode(YAW_INPUT_PIN,      INPUT);

    /* On each CHANGE on an input pin, an interrupt handler is called */
    attachInterrupt(THROTTLE_INPUT_PIN, read_throttle, CHANGE);
    attachInterrupt(ROLL_INPUT_PIN,     read_roll,     CHANGE);
    attachInterrupt(PITCH_INPUT_PIN,    read_pitch,    CHANGE);
    attachInterrupt(YAW_INPUT_PIN,      read_yaw,      CHANGE);

    delay(20);
    if (!has_signal_on_init()) {
        Serial.println("No receiver signal! Waiting.");
        while (!has_signal_on_init()) { };
    }
    Serial.println("Receiver signal detected, continuing.");
}


/*
   ————————————————————————————————————————————————————————————————
   ———             RECEIVER READ INTERRUPT ROUTINES             ———
   ————————————————————————————————————————————————————————————————
*/

void read_throttle() {
    if (digitalRead(THROTTLE_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[THROTTLE_CHANNEL] = micros();
    } else {
        receiver_in_shared[THROTTLE_CHANNEL] =
            (uint16_t)(micros() - receiver_pulse_start_time[THROTTLE_CHANNEL]);
    }
}

void read_roll() {
    if (digitalRead(ROLL_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[ROLL_CHANNEL] = micros();
    } else {
        receiver_in_shared[ROLL_CHANNEL] =
            (uint16_t)(micros() - receiver_pulse_start_time[ROLL_CHANNEL]);
    }
}

void read_pitch() {
    if (digitalRead(PITCH_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[PITCH_CHANNEL] = micros();
    } else {
        receiver_in_shared[PITCH_CHANNEL] =
            (uint16_t)(micros() - receiver_pulse_start_time[PITCH_CHANNEL]);
    }
}

void read_yaw() {
    if (digitalRead(YAW_INPUT_PIN) == HIGH) {
        receiver_pulse_start_time[YAW_CHANNEL] = micros();
    } else {
        receiver_in_shared[YAW_CHANNEL] =
            (uint16_t)(micros() - receiver_pulse_start_time[YAW_CHANNEL]);
    }
}
