#include "../include/ArduinoMock.h"
 
/*@ assigns milliseconds;
    behavior overflow:
    assumes milliseconds == UINT64_MAX;
    ensures milliseconds == 0;
    ensures \result == 0;

    behavior no_overflow:
    assumes milliseconds < UINT64_MAX;
    ensures milliseconds == \old(milliseconds) + 1;
    ensures \result == \old(milliseconds) + 1;

    complete behaviors overflow, no_overflow;
    disjoint behaviors overflow, no_overflow;
*/
uint64_t mock_millis() {
    if (milliseconds < UINT64_MAX) {
        milliseconds++;
    } else {
        milliseconds = 0;
    }
    return milliseconds;
}

/*@ assigns microseconds;
    behavior overflow:
    assumes microseconds == UINT64_MAX;
    ensures microseconds == 0;
    ensures \result == 0;

    behavior no_overflow:
    assumes microseconds < UINT64_MAX;
    ensures microseconds == \old(microseconds) + 1;
    ensures \result == \old(microseconds) + 1;

    complete behaviors overflow, no_overflow;
    disjoint behaviors overflow, no_overflow;
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

//@ assigns \nothing;
void mock_digitalWrite(int pin, int state) {

}

/*@ assigns \nothing;
    ensures \result == HIGH || \result == LOW;
*/
int mock_digitalRead(int pin) {
    return HIGH;
}

//@ assigns \nothing;
void mock_delay(int millis) {
    //implements: GLOBALnoDelayInLoop
    /*@ ghost if (!ghost_delay_allowed) {
            ghost_delay_happened = true;
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
