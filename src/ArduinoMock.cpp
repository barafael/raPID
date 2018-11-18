#include "../include/ArduinoMock.h"

#ifdef FRAMAC


static uint64_t microseconds;

static uint64_t milliseconds = 0;
/*@ ensures \result > \old(milliseconds);
    ensures milliseconds > \old(milliseconds);
    assigns milliseconds;
*/
uint64_t millis() {
    milliseconds++;
    return milliseconds;
}

uint64_t micros() {
    microseconds++;
    return microseconds;
}

void interrupts() {

}

void noInterrupts() {

}

#undef DOESNOTWORK
#ifdef DOESNOTWORK

/*@ ghost set<integer> ticks_millis = \empty; */
static unsigned long long milliseconds;

/*@ ghost set<integer> ticks_micros = \empty; */
static unsigned long long microseconds;

/* @ assigns ticks_millis;
   ensures ! \subset(\result, \old(\ticks_millis))
     && \result \in ticks_millis
     && \subset(\old(ticks_millis), ticks_millis);
 */
unsigned long long millis() {
    /* global invariant I: \forall integer k;
         k \in ticks_millis ==> milliseconds > k; */
    milliseconds++;
    //@ ghost ticks_millis = \union(milliseconds, ticks_millis);
    return milliseconds;
}

/* @ assigns ticks_micros;
   ensures ! \subset(\result, \old(\ticks_micros))
     && \result \in ticks_micros
     && \subset(\old(ticks_micros), ticks_micros);
 */
unsigned long long micros() {
    /* global invariant I: \forall integer k;
         k \in ticks_micros ==> microseconds > k; */
    microseconds++;
    //@ ghost ticks_micros = \union(microseconds, ticks_micros);
    return microseconds;
}
#endif
#endif // FRAMAC
