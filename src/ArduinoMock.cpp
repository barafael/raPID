#include "../include/ArduinoMock.h"

static int64_t milliseconds = 0;
static int64_t microseconds = 0;

/*@ assigns milliseconds;

    behavior normal:
      assumes milliseconds < INT64_MAX;
      ensures \result == \old(milliseconds) + 1;
      ensures milliseconds == \old(milliseconds) + 1;
    behavior overflow:
      assumes milliseconds == INT64_MAX;
      ensures \result == 0;
      ensures milliseconds == 0;

    complete behaviors normal, overflow;
    disjoint behaviors normal, overflow;
*/
int64_t mock_millis() {
    if (milliseconds < INT64_MAX) {
        milliseconds++;
    } else {
        milliseconds = 0;
    }
    return milliseconds;
}

/*@ assigns microseconds;

    behavior normal:
      assumes microseconds < INT64_MAX;
      ensures \result == \old(microseconds) + 1;
      ensures microseconds == \old(microseconds) + 1;
    behavior overflow:
      assumes microseconds == INT64_MAX;
      ensures \result == 0;
      ensures microseconds == 0;

    complete behaviors normal, overflow;
    disjoint behaviors normal, overflow;
*/
int64_t mock_micros() {
    if (microseconds < INT64_MAX) {
        microseconds++;
    } else {
        microseconds = 0;
    }
    return microseconds;
}

// assigning a ghost variable is not assigning \nothing
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

}

//@ assigns \nothing;
void mock_delayMicroseconds(uint64_t micros) {

}

//@ assigns \nothing;
void mock_pinMode(int pin, int mode) {

}

//@ assigns \nothing;
void attachInterrupt(int pin, void f(), int mode) {

}
