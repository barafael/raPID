#include "../../include/receiver/PWMReceiver.h"

static int throttle_pin;
static int roll_pin;
static int pitch_pin;
static int yaw_pin;
static int aux1_pin;
static int aux2_pin;

/* Written by interrupt on rising edge, read on falling edge */
static volatile uint64_t pwm_pulse_start_tick[NUM_CHANNELS] = { 0 };

/* Interrupts write to this array and the update function reads
 * Note: disable interrupts when reading to avoid race conditions */
static volatile uint64_t pwm_pulse_duration_shared[NUM_CHANNELS] = { 0 };

/*@ requires \valid(self);
    requires \valid(offsets + (0 .. 6 -1));
    requires \valid_read(offsets + (0 .. 6 -1));
    requires \valid(self->offsets + (0 .. 6 -1));
    requires \valid(self->pins + (0 .. 6 - 1));
    assigns *self;
    assigns *(self->offsets + (0 .. 6 - 1));
    ensures \forall int i; 0 <= i <= 6 - 1 ==> offsets[i] == self->offsets[i];
*/
void PWMReceiver_init(PWMReceiver_t *self, uint8_t _throttle_pin,
        uint8_t _roll_pin, uint8_t _pitch_pin, uint8_t _yaw_pin,
        uint8_t _aux1_pin, uint8_t _aux2_pin, const int16_t offsets[NUM_CHANNELS]) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->offsets[index] = offsets[index];
    }

    throttle_pin = _throttle_pin;
    roll_pin     = _roll_pin;
    pitch_pin    = _pitch_pin;
    yaw_pin      = _yaw_pin;
    aux1_pin     = _aux1_pin;
    aux2_pin     = _aux2_pin;

    attachInterrupt(throttle_pin, update_throttle, CHANGE);
    attachInterrupt(roll_pin,     update_roll,     CHANGE);
    attachInterrupt(pitch_pin,    update_pitch,    CHANGE);
    attachInterrupt(yaw_pin,      update_yaw,      CHANGE);
    attachInterrupt(aux1_pin,     update_aux1,     CHANGE);
    attachInterrupt(aux2_pin,     update_aux2,     CHANGE);
}

/*@
   requires \valid(self);
   requires \valid(channels);
   assigns \nothing;
   ensures \valid(\old(self)) ==> \valid(self);
   ensures interrupt_status == INTERRUPTS_ON;
*/
const void receiver_update(PWMReceiver_t *self, int16_t channels[NUM_CHANNELS]) {
    mock_noInterrupts();
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        channels[index] = pwm_pulse_duration_shared[index];
    }
    mock_interrupts();

    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        clamp(channels[index], 1000, 2000);
        if (self->inversion[index]) {
            channels[index] = 2000 - (channels[index] - 1000);
        }
        channels[index] += self->offsets[index];
        channels[index] += self->trims[index];
    }
}

void set_offsets(PWMReceiver_t *self, int16_t offsets[NUM_CHANNELS]) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->offsets[index] = offsets[index];
    }
}

void set_trims(PWMReceiver_t *self, int16_t trims[NUM_CHANNELS]) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->trims[index] = trims[index];
    }
}

void set_inversion(PWMReceiver_t *self, bool inversion[NUM_CHANNELS]) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        self->inversion[index] = inversion[index];
    }
}

// TODO copy has_signal logic from blinkenlights
/*@
   requires \valid(self);
   assigns \nothing;
   ensures \valid(\old(self)) ==> \valid(self);
   ensures interrupt_status == INTERRUPTS_ON;
*/
const bool has_signal() {
    mock_noInterrupts();
    for (size_t index = 0; index < 4; index++) {
        if (pwm_pulse_duration_shared[index] == 0) {
            mock_interrupts();
            return false;
        }
    }
    mock_interrupts();
    return true;
}

/*
   ----------------------------------------------------------------
   ---           PWMRECEIVER READ INTERRUPT ROUTINES            ---
   ----------------------------------------------------------------
*/

// is this sufficient to denote that this function cannot be called when interrupts are off?
/*@ requires interrupt_status == INTERRUPTS_ON;
*/
void update_throttle() {
    if (mock_digitalRead(throttle_pin) == HIGH) {
        pwm_pulse_start_tick[THROTTLE_CHANNEL] = mock_micros();
    } else {
        pwm_pulse_duration_shared[THROTTLE_CHANNEL] =
            (mock_micros() - pwm_pulse_start_tick[THROTTLE_CHANNEL]);
    }
}

void update_roll() {
    if (mock_digitalRead(roll_pin) == HIGH) {
        pwm_pulse_start_tick[ROLL_CHANNEL] = mock_micros();
    } else {
        pwm_pulse_duration_shared[ROLL_CHANNEL] =
            (mock_micros() - pwm_pulse_start_tick[ROLL_CHANNEL]);
    }
}

void update_pitch() {
    if (mock_digitalRead(pitch_pin) == HIGH) {
        pwm_pulse_start_tick[PITCH_CHANNEL] = mock_micros();
    } else {
        pwm_pulse_duration_shared[PITCH_CHANNEL] =
            (mock_micros() - pwm_pulse_start_tick[PITCH_CHANNEL]);
    }
}

void update_yaw() {
    if (mock_digitalRead(yaw_pin) == HIGH) {
        pwm_pulse_start_tick[YAW_CHANNEL] = mock_micros();
    } else {
        pwm_pulse_duration_shared[YAW_CHANNEL] =
            (mock_micros() - pwm_pulse_start_tick[YAW_CHANNEL]);
    }
}

void update_aux1() {
    if (mock_digitalRead(aux1_pin) == HIGH) {
        pwm_pulse_start_tick[AUX1_CHANNEL] = mock_micros();
    } else {
        pwm_pulse_duration_shared[AUX1_CHANNEL] =
            (mock_micros() - pwm_pulse_start_tick[AUX1_CHANNEL]);
    }
}

void update_aux2() {
    if (mock_digitalRead(aux2_pin) == HIGH) {
        pwm_pulse_start_tick[AUX2_CHANNEL] = mock_micros();
    } else {
        pwm_pulse_duration_shared[AUX2_CHANNEL] =
            (mock_micros() - pwm_pulse_duration_shared[AUX2_CHANNEL]);
    }
}
