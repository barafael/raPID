#include "../include/Mock.h"

#ifdef BINARY
#include<Arduino.h>
#include<i2c_t3.h>

uint64_t mock_millis() {
    return millis();
}

uint64_t mock_micros() {
    return micros();
}

void mock_interrupts() {
    interrupts();
}

void mock_noInterrupts() {
    noInterrupts();
}

void mock_analogWrite(int pin, int value) {
    analogWrite(pin, value);
}

void mock_digitalWrite(int pin, int state) {
    digitalWrite(pin, state);
}

int mock_digitalRead(int pin) {
    return digitalRead(pin);
}

void mock_delay(int millis) {
    delay(millis);
}

//@ assigns \nothing;
void mock_delayMicroseconds(uint64_t micros) {

}

void mock_pinMode(int pin, int mode) {
    pinMode(pin, mode);
}

void mock_attachInterrupt(int pin, void f(), int mode) {
    attachInterrupt(pin, f, mode);
}

int wire_requestFrom(int devAddr, int size) {
    return Wire.requestFrom(devAddr, size);
}

void wire_beginTransmission(int device) {
    Wire.beginTransmission(device);
}

void wire_write(int address) {
    Wire.write(address);
}

int wire_read() {
    return Wire.read();
}

int wire_endTransmission() {
    return Wire.endTransmission();
}

int wire_endTransmission_nostop() {
    return Wire.endTransmission(I2C_NOSTOP);
}

int wire_available() {
    return Wire.available();
}

void wire_begin(i2c_mode mode, uint8_t address, i2c_pins pins, i2c_pullup pullup, uint32_t rate, i2c_op_mode opMode) {
    Wire.begin(mode, address, pins, pullup, rate, opMode);
}

#endif // BINARY

#ifdef FRAMAC

/*@ assigns milliseconds;
    behavior GLOBAL_elapsed_time_calculation:
        assumes milliseconds == UINT64_MAX;
        ensures milliseconds == 0;
        ensures \result == 0;

    behavior no_overflow:
        assumes milliseconds < UINT64_MAX;
        ensures milliseconds == \old(milliseconds) + 1;
        ensures \result == \old(milliseconds) + 1;

    complete behaviors GLOBAL_elapsed_time_calculation, no_overflow;
    disjoint behaviors GLOBAL_elapsed_time_calculation, no_overflow;
*/
uint64_t mock_millis() {
    //implements: GLOBAL_elapsed_time: GLOBAL_elapsed_time_calculation
    if (milliseconds < UINT64_MAX) {
        milliseconds++;
    } else {
        milliseconds = 0;
    }
    return milliseconds;
}

/*@ assigns microseconds;
    behavior GLOBAL_elapsed_time_calculation:
    assumes microseconds == UINT64_MAX;
    ensures microseconds == 0;
    ensures \result == 0;

    behavior no_overflow:
    assumes microseconds < UINT64_MAX;
    ensures microseconds == \old(microseconds) + 1;
    ensures \result == \old(microseconds) + 1;

    complete behaviors GLOBAL_elapsed_time_calculation, no_overflow;
    disjoint behaviors GLOBAL_elapsed_time_calculation, no_overflow;
*/
uint64_t mock_micros() {
    if (microseconds < UINT64_MAX) {
        microseconds++;
    } else {
        microseconds = 0;
    }
    return microseconds;
}

/*@ ensures ghost_interrupt_status == INTERRUPTS_ON;
    assigns ghost_interrupt_status; */
void mock_interrupts() {
    //@ ghost ghost_interrupt_status = INTERRUPTS_ON;
}

/*@ ensures ghost_interrupt_status == INTERRUPTS_OFF;
    assigns ghost_interrupt_status; */
void mock_noInterrupts() {
    //@ ghost ghost_interrupt_status = INTERRUPTS_OFF;
}

void mock_analogWrite(int pin, int value);

//@ assigns \result \from \nothing;
int mock_digitalRead(int pin);

//@ assigns \nothing;
void mock_digitalWrite(int pin, int state) {

}

//@ assigns ghost_delay_happened;
void mock_delay(int millis) {
    //implements: GLOBAL_no_delay_in_loop
    /*@ ghost if (!ghost_delay_allowed) {
            ghost_delay_happened = 1;
        }
    */
}

//@ assigns \nothing;
void mock_delayMicroseconds(uint64_t micros) {

}

//@ assigns \nothing;
void mock_pinMode(int pin, int mode) {

}

//@ assigns \nothing;
void mock_attachInterrupt(int pin, void f(), int mode) {

}

//implements: MAIN_no_alloc
void *mock_malloc(size_t s) {
    *(volatile char*)NULL = 1;
    return (void*) NULL;
}

#endif // FRAMAC
