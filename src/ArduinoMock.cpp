#include "../include/ArduinoMock.h"
 
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

//@ assigns \nothing;
void mock_digitalWrite(int pin, int state) {

}

/*@ assigns \nothing;
    ensures \result == HIGH || \result == LOW; */
int mock_digitalRead(int pin) {
    return HIGH;
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
