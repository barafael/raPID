#include "Arduino.h"
#include "WProgram.h"

#include "../interface/receiver.h"

/* Access variable for ISRs */
static Receiver *instance = NULL;

typedef enum {
    THROTTLE_CHANNEL = 0,
    AILERON_CHANNEL  = 1,
    ELEVATOR_CHANNEL = 2,
    RUDDER_CHANNEL   = 3,
    AUX1_CHANNEL     = 4,
    AUX2_CHANNEL     = 5
} input_channel;

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

void update_aileron() {
    if (digitalRead(instance->aileron_pin) == HIGH) {
        instance->receiver_pulse_start_time[AILERON_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[AILERON_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[AILERON_CHANNEL]);
    }
}

void update_elevator() {
    if (digitalRead(instance->elevator_pin) == HIGH) {
        instance->receiver_pulse_start_time[ELEVATOR_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[ELEVATOR_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[ELEVATOR_CHANNEL]);
    }
}

void update_rudder() {
    if (digitalRead(instance->rudder_pin) == HIGH) {
        instance->receiver_pulse_start_time[RUDDER_CHANNEL] = micros();
    } else {
        instance->receiver_in_shared[RUDDER_CHANNEL] =
            (uint16_t)(micros() - instance->receiver_pulse_start_time[RUDDER_CHANNEL]);
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

Receiver::Receiver(uint8_t _throttle_pin, uint8_t _aileron_pin,
                   uint8_t _elevator_pin, uint8_t _rudder_pin,
                   uint8_t _aux1_pin,     uint8_t _aux2_pin) {

    throttle_pin = _throttle_pin;
    aileron_pin  = _aileron_pin;
    elevator_pin = _elevator_pin;
    rudder_pin   = _rudder_pin;
    aux1_pin     = _aux1_pin;
    aux2_pin     = _aux2_pin;

    /* The pinMode should be set to input by default, set it anyway */
    pinMode(throttle_pin, INPUT);
    pinMode(aileron_pin,  INPUT);
    pinMode(elevator_pin, INPUT);
    pinMode(rudder_pin,   INPUT);
    pinMode(aux1_pin,     INPUT);
    pinMode(aux2_pin,     INPUT);

    /* On each CHANGE on an input pin, an interrupt handler is called */
    attachInterrupt(throttle_pin, update_throttle, CHANGE);
    attachInterrupt(aileron_pin,  update_aileron,  CHANGE);
    attachInterrupt(elevator_pin, update_elevator, CHANGE);
    attachInterrupt(rudder_pin,   update_rudder,   CHANGE);
    attachInterrupt(aux1_pin,     update_aux1,     CHANGE);
    attachInterrupt(aux2_pin,     update_aux2,     CHANGE);

    /* TODO Check if delay necessary for has_signal */
    delay(10);
}

void Receiver::get_channels(uint16_t channels[NUM_CHANNELS]) {
    channels[THROTTLE_CHANNEL] = get_throttle();
    channels[AILERON_CHANNEL]  = get_aileron();
    channels[ELEVATOR_CHANNEL] = get_elevator();
    channels[RUDDER_CHANNEL]   = get_rudder();
    channels[AUX1_CHANNEL]     = get_aux1();
    channels[AUX2_CHANNEL]     = get_aux2();
}

uint16_t Receiver::get_throttle() {
    return receiver_in[THROTTLE_CHANNEL];
}

uint16_t Receiver::get_aileron() {
    return receiver_in[AILERON_CHANNEL];
}

uint16_t Receiver::get_elevator() {
    return receiver_in[ELEVATOR_CHANNEL];
}

uint16_t Receiver::get_rudder() {
    return receiver_in[RUDDER_CHANNEL];
}

uint16_t Receiver::get_aux1() {
    return receiver_in[AUX1_CHANNEL];
}

uint16_t Receiver::get_aux2() {
    return receiver_in[AUX2_CHANNEL];
}


/*
   —————————————————————————————————————————————————————————
   ———             RECEIVER UPDATE FUNCTION                ———
   —————————————————————————————————————————————————————————
*/

void Receiver::update() {
    noInterrupts();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        receiver_in[index] = receiver_in_shared[index];
    }
    interrupts();

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        if (receiver_in[index] < 1000) receiver_in[index] = 1000;
        if (receiver_in[index] > 2000) receiver_in[index] = 2000;
    }
}

void Receiver::update(uint16_t channels[NUM_CHANNELS]) {
    this->update();
    this->get_channels(channels);
}

bool Receiver::has_signal() {
    this->update();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        if (receiver_in[index] == 0) {
            return false;
        }
    }
    return true;
}

