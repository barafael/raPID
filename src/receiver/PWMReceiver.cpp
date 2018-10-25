#include "Arduino.h"

#include "../../include/receiver/PWMReceiver.h"

/* Access variable for ISRs */
extern PWMReceiver_t *receiver_instance;

/* Array of void functions without params, one for each input */
static void (*interrupts[NUM_CHANNELS])() = {};

PWMReceiver_t PWMReceiver_init(uint8_t throttle_pin, uint8_t roll_pin, uint8_t pitch_pin, uint8_t yaw_pin,
        uint8_t aux1_pin, uint8_t aux2_pin,
        channels_t offsets) {
    PWMReceiver_t self;

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self.offsets[index] = offsets[index];
    }

    interrupts[0] = update_throttle;
    interrupts[1] = update_roll;
    interrupts[2] = update_pitch;
    interrupts[3] = update_yaw;
    interrupts[4] = update_aux1;
    interrupts[5] = update_aux2;

    self.pins[0] = throttle_pin;
    self.pins[1] = roll_pin;
    self.pins[2] = pitch_pin;
    self.pins[3] = yaw_pin;
    self.pins[4] = aux1_pin;
    self.pins[5] = aux2_pin;

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        attachInterrupt(self.pins[index], interrupts[index], CHANGE);
    }

    return self;
}


/*
   ---------------------------------------------------------
   ---           PWMRECEIVER UPDATE FUNCTION             ---
   ---------------------------------------------------------
*/

const void update(PWMReceiver_t *self, channels_t channels) {
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

void set_offsets(PWMReceiver_t *self, channels_t offsets) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->offsets[index] = offsets[index];
    }
}

void set_trims(PWMReceiver_t *self, channels_t trims) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->trims[index] = trims[index];
    }
}

void set_inversion(PWMReceiver_t *self, inversion_t inversion) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->inversion[index] = inversion[index];
    }
}

const bool has_signal(PWMReceiver_t *self) {
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
