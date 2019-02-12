#include "../../include/receiver/pwm_receiver.h"
#include "./../../include/copy_int16.h"
#include "../../include/copy_bool.h"

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

/*@ requires valid_access: \valid(_offsets + (0 .. NUM_CHANNELS - 1));
    requires valid_access: \valid(pwm_offsets + (0 .. NUM_CHANNELS - 1));
    requires valid_separation: \separated(pwm_offsets + (0 .. NUM_CHANNELS - 1), _offsets + (0 .. NUM_CHANNELS - 1));

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

    ensures throttle_pin == _throttle_pin;
    ensures roll_pin     == _roll_pin;
    ensures pitch_pin    == _pitch_pin;
    ensures yaw_pin      == _yaw_pin;
    ensures aux1_pin     == _aux1_pin;
    ensures aux2_pin     == _aux2_pin;
    ensures RXTXinitialization: ghost_pwmreceiver_status == PWM_RECEIVER_INITIALIZED;
*/
void pwm_receiver_init(uint8_t _throttle_pin,
        uint8_t _roll_pin, uint8_t _pitch_pin, uint8_t _yaw_pin,
        uint8_t _aux1_pin, uint8_t _aux2_pin, const int16_t _offsets[NUM_CHANNELS]) {
    set_offsets(_offsets);

    throttle_pin = _throttle_pin;
    roll_pin     = _roll_pin;
    pitch_pin    = _pitch_pin;
    yaw_pin      = _yaw_pin;
    aux1_pin     = _aux1_pin;
    aux2_pin     = _aux2_pin;

    // RXTXregisterInterrupts:
    mock_attachInterrupt(throttle_pin, update_throttle, CHANGE);
    mock_attachInterrupt(roll_pin,     update_roll,     CHANGE);
    mock_attachInterrupt(pitch_pin,    update_pitch,    CHANGE);
    mock_attachInterrupt(yaw_pin,      update_yaw,      CHANGE);
    mock_attachInterrupt(aux1_pin,     update_aux1,     CHANGE);
    mock_attachInterrupt(aux2_pin,     update_aux2,     CHANGE);
    //@ ghost ghost_pwmreceiver_status = PWM_RECEIVER_INITIALIZED;
}

/*@
   requires RXTXinitialization: ghost_pwmreceiver_status == PWM_RECEIVER_INITIALIZED;
   requires valid_access: \valid(channels + (0 .. NUM_CHANNELS - 1));
   requires valid_access: \valid_read(pwm_pulse_duration_shared + (0 .. NUM_CHANNELS - 1));

   requires valid_access: \valid_read(pwm_offsets + (0 .. NUM_CHANNELS - 1));
   requires valid_access: \valid_read(trims + (0 .. NUM_CHANNELS - 1));
   requires valid_access: \valid_read(inversion + (0 .. NUM_CHANNELS - 1));

   requires valid_separation: \separated(
   pwm_pulse_duration_shared + (0 .. NUM_CHANNELS - 1),
   channels + (0 .. NUM_CHANNELS - 1),
   pwm_offsets + (0 .. NUM_CHANNELS - 1),
   inversion + (0 .. NUM_CHANNELS - 1), 
   trims + (0 .. NUM_CHANNELS - 1));

   assigns channels[0];
   assigns channels[1];
   assigns channels[2];
   assigns channels[3];
   assigns channels[4];
   assigns channels[5];
   assigns ghost_interrupt_status;

   ensures RXTXinterruptSafety: ghost_interrupt_status == INTERRUPTS_ON;
*/
const void receiver_update(int16_t channels[NUM_CHANNELS]) {
    mock_noInterrupts();
    //@ assert RXTXinterruptSafety: ghost_interrupt_status == INTERRUPTS_OFF;
    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        //loop invariant \forall int j \in (0 .. index) ==> channels[j] == pwm_pulse_duration_shared[j];
        loop assigns index, channels[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        channels[index] = pwm_pulse_duration_shared[index];
    }
    mock_interrupts();
    //@ assert RXTXinterruptSafety: ghost_interrupt_status == INTERRUPTS_ON;

pre_inversion:
    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop invariant \forall integer i; 0 <= i < index ==> !inversion[i] ==> channels[i] == \at(channels[i], pre_inversion);
        loop invariant \forall integer i; 0 <= i < index ==> inversion[i] ==> channels[i] == 2000 - (\at(channels[i], pre_inversion)) - 1000);
        loop assigns index, channels[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        if (inversion[index]) {
            channels[index] = 2000 - (channels[index] - 1000);
        }
    }

    // only a bounding condition
    //@ assert RXTXboundedResults: 1000 <= channels[0] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[1] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[2] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[3] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[4] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[5] <= 2000;

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

    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns index, channels[0 .. (index - 1)];
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        clamp(channels[index], 1000, 2000);
    }

    //@ assert RXTXboundedResults: 1000 <= channels[0] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[1] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[2] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[3] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[4] <= 2000;
    //@ assert RXTXboundedResults: 1000 <= channels[5] <= 2000;
}

/*@ requires valid_access: \valid(_offsets + (0 .. NUM_CHANNELS - 1));
    requires valid_access: \valid(pwm_offsets + (0 .. NUM_CHANNELS - 1));

    requires valid_separation: \separated(pwm_offsets + (0 .. NUM_CHANNELS − 1) , _offsets + (0 .. NUM_CHANNELS − 1));

   requires RXTXinitialization: ghost_pwmreceiver_status == PWM_RECEIVER_INITIALIZED;

    assigns pwm_offsets[0];
    assigns pwm_offsets[1];
    assigns pwm_offsets[2];
    assigns pwm_offsets[3];
    assigns pwm_offsets[4];
    assigns pwm_offsets[5];

    ensures IsEqual_int16{Here, Here}(&pwm_offsets[0], NUM_CHANNELS, &_offsets[0]) ;

    ensures _offsets[0] == pwm_offsets[0];
    ensures _offsets[1] == pwm_offsets[1];
    ensures _offsets[2] == pwm_offsets[2];
    ensures _offsets[3] == pwm_offsets[3];
    ensures _offsets[4] == pwm_offsets[4];
    ensures _offsets[5] == pwm_offsets[5];
*/
void set_offsets(const int16_t _offsets[NUM_CHANNELS]) {
    copy_int16(_offsets, pwm_offsets, NUM_CHANNELS);
}

/*@ requires valid_access: \valid(trims + (0 .. NUM_CHANNELS - 1));
    requires valid_access: \valid(_trims + (0 .. NUM_CHANNELS - 1));

    requires valid_separation: \separated(trims + (0 .. NUM_CHANNELS − 1) , _trims + (0 .. NUM_CHANNELS − 1));

   requires RXTXinitialization: ghost_pwmreceiver_status == PWM_RECEIVER_INITIALIZED;

    assigns trims[0 .. NUM_CHANNELS - 1];

    ensures IsEqual_int16{Here, Here}(&trims[0], NUM_CHANNELS, &_trims[0]) ;
    */
void set_trims(const int16_t _trims[NUM_CHANNELS]) {
    copy_int16(_trims, trims, NUM_CHANNELS);
}

/*@ requires valid_access: \valid(inversion + (0 .. NUM_CHANNELS - 1));
    requires valid_access: \valid(_inversion + (0 .. NUM_CHANNELS - 1));

    requires valid_separation: \separated(inversion + (0 .. NUM_CHANNELS − 1) , _inversion + (0 .. NUM_CHANNELS − 1));

    assigns inversion[0 .. NUM_CHANNELS - 1];

    ensures IsEqual_bool{Here, Here}(&inversion[0], NUM_CHANNELS, &_inversion[0]) ;

    ensures inversion[0] == _inversion[0];
    ensures inversion[1] == _inversion[1];
    ensures inversion[2] == _inversion[2];
    ensures inversion[3] == _inversion[3];
    ensures inversion[4] == _inversion[4];
    ensures inversion[5] == _inversion[5];
*/
void set_inversion(const bool _inversion[NUM_CHANNELS]) {
    copy_bool(_inversion, inversion, NUM_CHANNELS);
}

// TODO use pwm_pulse_start_tick to calculate time since last event
/*@
   requires \valid(pwm_pulse_duration_shared + (0 .. NUM_CHANNELS - 1));

   requires RXTXinitialization: ghost_pwmreceiver_status == PWM_RECEIVER_INITIALIZED;

   assigns ghost_interrupt_status;
   behavior has_connection:
     assumes \forall integer i; 0 <= i < NUM_CHANNELS ==> pwm_pulse_duration_shared[i] <= 80000;
     ensures ghost_interrupt_status == INTERRUPTS_ON;
     ensures \result == true;
   behavior RXTXconnectionLoss:
     assumes \exists integer i; 0 <= i < NUM_CHANNELS ==> pwm_pulse_duration_shared[i] > 80000;
     ensures ghost_interrupt_status == INTERRUPTS_ON;
     ensures \result == false;
   
   complete behaviors has_connection, RXTXconnectionLoss;
   disjoint behaviors has_connection, RXTXconnectionLoss;
*/
const bool has_signal() {
    mock_noInterrupts();
    //@ assert RXTXinterruptSafety: ghost_interrupt_status == INTERRUPTS_OFF;
    /*@ loop invariant 0 <= index <= NUM_CHANNELS;
        loop assigns ghost_interrupt_status, index;
        loop variant NUM_CHANNELS - index;
    */
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        // assume no signal if no pulse for 80 milliseconds
        // RXTXconnectionTimeout:
        if (pwm_pulse_duration_shared[index] > 80000) {
            mock_interrupts();
            //@ assert RXTXinterruptSafety: ghost_interrupt_status == INTERRUPTS_ON;
            return false;
        }
    }
    mock_interrupts();
    //@ assert RXTXinterruptSafety: ghost_interrupt_status == INTERRUPTS_ON;
    return true;
}

/*
   ----------------------------------------------------------------
   ---           PWMRECEIVER READ INTERRUPT ROUTINES            ---
   ----------------------------------------------------------------
*/

// is this sufficient to denote that this function will never be called when interrupts are off?
/*@ requires RXTXinterruptSafety: ghost_interrupt_status == INTERRUPTS_ON;
   requires RXTXinitialization: ghost_pwmreceiver_status == PWM_RECEIVER_INITIALIZED;
    requires \valid(pwm_pulse_start_tick + (0 .. NUM_CHANNELS - 1));
    requires \valid(pwm_pulse_duration_shared + (0 .. NUM_CHANNELS - 1));
    // impossible to prove
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
