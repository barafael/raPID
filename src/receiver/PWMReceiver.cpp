#include "Arduino.h"

#include "../../include/receiver/pwm_receiver.h"

/* Access variable for ISRs */
static pwm_receiver_t *receiver_instance = NULL;

/* Array of void functions without params, one for each input */
static void (*interrupts[NUM_CHANNELS])() = {};

void pwm_receiver_init(pwm_receiver_t *self, uint8_t throttle_pin,
        uint8_t roll_pin, uint8_t pitch_pin, uint8_t yaw_pin,
        uint8_t aux1_pin, uint8_t aux2_pin, int16_t *offsets) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->offsets[index] = offsets[index];
    }

    interrupts[0] = update_throttle;
    interrupts[1] = update_roll;
    interrupts[2] = update_pitch;
    interrupts[3] = update_yaw;
    interrupts[4] = update_aux1;
    interrupts[5] = update_aux2;

    self->pins[0] = throttle_pin;
    self->pins[1] = roll_pin;
    self->pins[2] = pitch_pin;
    self->pins[3] = yaw_pin;
    self->pins[4] = aux1_pin;
    self->pins[5] = aux2_pin;

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        attachInterrupt(self->pins[index], interrupts[index], CHANGE);
    }
}

/*
   ---------------------------------------------------------
   ---           PWMRECEIVER UPDATE FUNCTION             ---
   ---------------------------------------------------------
*/

const void receiver_update(pwm_receiver_t *self, int16_t *channels) {
    noInterrupts();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        channels[index] = self->channels_shared[index];
    }
    interrupts();

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        clamp(channels[index], 1000, 2000);
        if (self->inversion[index]) {
            channels[index] = 2000 - (channels[index] - 1000);
        }
        channels[index] += self->offsets[index];
        channels[index] += self->trims[index];
    }
}

void set_offsets(pwm_receiver_t *self, int16_t *offsets) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->offsets[index] = offsets[index];
    }
}

void set_trims(pwm_receiver_t *self, int16_t *trims) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->trims[index] = trims[index];
    }
}

void set_inversion(pwm_receiver_t *self, bool inversion[NUM_CHANNELS]) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->inversion[index] = inversion[index];
    }
}

// TODO copy has_signal logic from blinkenlights
// Prove: interrupts always enabled after this function
const bool has_signal(pwm_receiver_t *self) {
    noInterrupts();
    for (size_t index = 0; index < 4; index++) {
        if (self->channels_shared[index] == 0) {
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
    if (digitalRead(receiver_instance->pins[0]) == HIGH) {
        receiver_instance->pwm_pulse_start_time[THROTTLE_CHANNEL] = micros();
    } else {
        receiver_instance->channels_shared[THROTTLE_CHANNEL] =
            (uint16_t)(micros() - receiver_instance->pwm_pulse_start_time[THROTTLE_CHANNEL]);
    }
}

void update_roll() {
    if (digitalRead(receiver_instance->pins[1]) == HIGH) {
        receiver_instance->pwm_pulse_start_time[ROLL_CHANNEL] = micros();
    } else {
        receiver_instance->channels_shared[ROLL_CHANNEL] =
            (uint16_t)(micros() - receiver_instance->pwm_pulse_start_time[ROLL_CHANNEL]);
    }
}

void update_pitch() {
    if (digitalRead(receiver_instance->pins[2]) == HIGH) {
        receiver_instance->pwm_pulse_start_time[PITCH_CHANNEL] = micros();
    } else {
        receiver_instance->channels_shared[PITCH_CHANNEL] =
            (uint16_t)(micros() - receiver_instance->pwm_pulse_start_time[PITCH_CHANNEL]);
    }
}

void update_yaw() {
    if (digitalRead(receiver_instance->pins[3]) == HIGH) {
        receiver_instance->pwm_pulse_start_time[YAW_CHANNEL] = micros();
    } else {
        receiver_instance->channels_shared[YAW_CHANNEL] =
            (uint16_t)(micros() - receiver_instance->pwm_pulse_start_time[YAW_CHANNEL]);
    }
}

void update_aux1() {
    if (digitalRead(receiver_instance->pins[4]) == HIGH) {
        receiver_instance->pwm_pulse_start_time[AUX1_CHANNEL] = micros();
    } else {
        receiver_instance->channels_shared[AUX1_CHANNEL] =
            (uint16_t)(micros() - receiver_instance->pwm_pulse_start_time[AUX1_CHANNEL]);
    }
}

void update_aux2() {
    if (digitalRead(receiver_instance->pins[5]) == HIGH) {
        receiver_instance->pwm_pulse_start_time[AUX2_CHANNEL] = micros();
    } else {
        receiver_instance->channels_shared[AUX2_CHANNEL] =
            (uint16_t)(micros() - receiver_instance->pwm_pulse_start_time[AUX2_CHANNEL]);
    }
}
