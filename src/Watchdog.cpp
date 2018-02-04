#include "../include/Watchdog.h"

#define NO_WATCHDOG
Watchdog::Watchdog(int prescale) {
#ifdef NO_WATCHDOG
    return;
#endif
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);

    /* Enable WDG */
    WDOG_STCTRLH = 0x0001;

    /* The next 2 lines sets the time-out value.
     * This is the value that the watchdog timer compare itself to.
     */
    WDOG_TOVALL = 200;
    WDOG_TOVALH = 0;

    /* This sets prescale clock so that the watchdog timer ticks at 100Hz.
     * Formula: 1kHZ/4 = 200 HZ
     */
    WDOG_PRESC = 4;
}

void Watchdog::feed() {
#ifdef NO_WATCHDOG
    return;
#endif
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}
