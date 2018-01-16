#include "Arduino.h"
#include "WProgram.h"

#include "../include/watchdog.h"

void init_watchdog() {
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);
    /* Enable WDG */
    WDOG_STCTRLH = 0x0001;
    /* The next 2 lines sets the time-out value.
     * This is the value that the watchdog timer compare itself to.
     * */
    WDOG_TOVALL = 200;
    WDOG_TOVALH = 0;
    /* This sets prescale clock so that the watchdog timer ticks at
     * 1kHZ instead of the default 1kHZ/4 = 200 HZ
     * */
    WDOG_PRESC = 4;
}

void feed_the_dog() {
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}
