#include "../../include/receiver/pwm_receiver.h"

#define NUM_CHANNELS 6

static uint8_t throttle_pin;
static uint8_t roll_pin;
static uint8_t pitch_pin;
static uint8_t yaw_pin;
static uint8_t aux1_pin;
static uint8_t aux2_pin;

/* Written by interrupt on rising edge, read on falling edge */
static volatile uint64_t pwm_pulse_start_tick[NUM_CHANNELS] = { 0 };

/* Interrupts write to this array and the update function reads
 * Note: disable interrupts when reading to avoid race conditions */
static volatile uint64_t pwm_pulse_duration_shared[NUM_CHANNELS] = { 0 };

/*@ requires \valid_read(_offsets + (0 .. NUM_CHANNELS - 1));
    requires \valid(pwm_offsets + (0 .. NUM_CHANNELS - 1));
    // sometimes, it is quite hard to intuitively understand what frama-c still needs for the proof. 
    // It makes sense that the pointers should be separated! But it's hard to understand the first time.
    requires \separated(pwm_offsets, _offsets);
    assigns pwm_offsets[0];
    assigns pwm_offsets[1];
    assigns pwm_offsets[2];
    assigns pwm_offsets[3];
    assigns pwm_offsets[4];
    assigns pwm_offsets[5];

    assigns throttle_pin;
    assigns roll_pin;
    assigns pitch_pin;
    assigns yaw_pin;
    assigns aux1_pin;
    assigns aux2_pin;
    assigns ghost_pwmreceiver_status;

    ensures \forall size_t i; 0 <= i < NUM_CHANNELS ==> _offsets[i] == pwm_offsets[i];
    ensures pwm_offsets[0] == _offsets[0];
    ensures pwm_offsets[1] == _offsets[1];
    ensures pwm_offsets[2] == _offsets[2];
    ensures pwm_offsets[3] == _offsets[3];
    ensures pwm_offsets[4] == _offsets[4];
    ensures pwm_offsets[5] == _offsets[5];
    ensures throttle_pin == _throttle_pin;
    ensures roll_pin     == _roll_pin;
    ensures pitch_pin    == _pitch_pin;
    ensures yaw_pin      == _yaw_pin;
    ensures aux1_pin     == _aux1_pin;
    ensures aux2_pin     == _aux2_pin;
    ensures ghost_pwmreceiver_status == PWM_RECEIVER_INITIALIZED;
*/
void pwm_receiver_init(uint8_t _throttle_pin,
        uint8_t _roll_pin, uint8_t _pitch_pin, uint8_t _yaw_pin,
        uint8_t _aux1_pin, uint8_t _aux2_pin, const int16_t _offsets[NUM_CHANNELS]) {
    pwm_offsets[0] = _offsets[0];
    pwm_offsets[1] = _offsets[1];
    pwm_offsets[2] = _offsets[2];
    pwm_offsets[3] = _offsets[3];
    pwm_offsets[4] = _offsets[4];
    pwm_offsets[5] = _offsets[5];

    // breaks proofs:
    /*//@ loop invariant 0 <= index <= NUM_CHANNELS;
        //loop invariant \forall int j < index ==> 0 <= pwm_offsets[
        loop assigns index, pwm_offsets[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    /*
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        pwm_offsets[index] = offsets[index];
    }
    */

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
    //@ ghost ghost_pwmreceiver_status = PWM_RECEIVER_INITIALIZED;
}

/*@
   requires \valid(channels);
   requires \valid_read(pwm_pulse_duration_shared + (0 .. NUM_CHANNELS));
   requires \valid(channels + (0 .. NUM_CHANNELS - 1));
   requires ghost_pwmreceiver_status == PWM_RECEIVER_INITIALIZED;

   requires \valid_read(pwm_offsets + (0 .. NUM_CHANNELS));
   requires \valid_read(trims + (0 .. NUM_CHANNELS));
   requires \valid_read(inversion + (0 .. NUM_CHANNELS));

   requires \separated(pwm_channels, channels);

   assigns channels[0];
   assigns channels[1];
   assigns channels[2];
   assigns channels[3];
   assigns channels[4];
   assigns channels[5];
   assigns interrupt_status;

   ensures ghost_interrupt_status == INTERRUPTS_ON;
*/
const void receiver_update(int16_t channels[NUM_CHANNELS]) {
    mock_noInterrupts();
    //@ assert ghost_interrupt_status == INTERRUPTS_OFF;
    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        //loop invariant \forall int j \in (0 .. index) ==> channels[j] == pwm_pulse_duration_shared[j];
        loop assigns index, channels[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        channels[index] = pwm_pulse_duration_shared[index];
    }
    mock_interrupts();
    //@ assert ghost_interrupt_status == INTERRUPTS_ON;

    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns index, channels[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        clamp(channels[index], 1000, 2000);
    }

    //@ assert 1000 <= channels[0] <= 2000;
    //@ assert 1000 <= channels[1] <= 2000;
    //@ assert 1000 <= channels[2] <= 2000;
    //@ assert 1000 <= channels[3] <= 2000;
    //@ assert 1000 <= channels[4] <= 2000;
    //@ assert 1000 <= channels[5] <= 2000;

    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns index, channels[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        if (inversion[index]) {
            channels[index] = 2000 - (channels[index] - 1000);
        }
    }

    // not a strict assertion
    //@ assert 1000 <= channels[0] <= 2000;
    //@ assert 1000 <= channels[1] <= 2000;
    //@ assert 1000 <= channels[2] <= 2000;
    //@ assert 1000 <= channels[3] <= 2000;
    //@ assert 1000 <= channels[4] <= 2000;
    //@ assert 1000 <= channels[5] <= 2000;

    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns index, channels[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        channels[index] += pwm_offsets[index];
    }

    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns index, channels[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        channels[index] += trims[index];
    }
}

void set_offsets(int16_t _offsets[NUM_CHANNELS]) {
    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns index, pwm_offsets[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        pwm_offsets[index] = _offsets[index];
    }
}

void set_trims(int16_t _trims[NUM_CHANNELS]) {
    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns index, trims[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        trims[index] = _trims[index];
    }
}

void set_inversion(bool _inversion[NUM_CHANNELS]) {
    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns index, inversion[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        inversion[index] = _inversion[index];
    }
}

// TODO copy has_signal logic from blinkenlights
/*@
   requires \valid(pwm_pulse_duration_shared + (0 .. NUM_CHANNELS - 1));
   assigns \nothing;
   ensures ghost_interrupt_status == INTERRUPTS_ON;
*/
const bool has_signal() {
    mock_noInterrupts();
    //@ assert interrupt_status == INTERRUPTS_OFF;
    for (size_t index = 0; index < 4; index++) {
        if (pwm_pulse_duration_shared[index] == 0) {
            mock_interrupts();
            //@ assert interrupt_status == INTERRUPTS_ON;
            return false;
        }
    }
    mock_interrupts();
    //@ assert interrupt_status == INTERRUPTS_ON;
    return true;
}

/*
   ----------------------------------------------------------------
   ---           PWMRECEIVER READ INTERRUPT ROUTINES            ---
   ----------------------------------------------------------------
*/

// is this sufficient to denote that this function will never be called when interrupts are off?
/*@ requires ghost_interrupt_status == INTERRUPTS_ON;
    requires \valid(pwm_pulse_start_tick + (0 .. NUM_CHANNELS));
    requires \valid(pwm_pulse_duration_shared + (0 .. NUM_CHANNELS));
    // absolutely impossible to prove
    // ensure 1000 <= pwm_pulse_start_tick[THROTTLE_CHANNEL] <= 2000;
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
