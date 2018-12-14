#include "../include/watchdog.h"

/*@ ensures WDOG_UNLOCK == WDOG_UNLOCK_SEQ2;
    ensures WDOG_PRESC == 4 */
void init_watchdog() {
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    mock_delayMicroseconds(1);

    /* Enable WDG */
    WDOG_STCTRLH = (uint16_t) 0x0001;

    /* The next 2 lines sets the time-out value.
     * This is the value that the watchdog timer compare itself to.
     */
    WDOG_TOVALL = (uint16_t) 200;
    WDOG_TOVALH = (uint16_t) 0;

    /* This sets prescale clock so that the watchdog timer ticks at 100Hz.
     * Formula: 1kHZ/4 = 200 HZ
     */
    WDOG_PRESC = (uint16_t) 4;
}

/*@ ensures WDOG_PRESC == 4 */
void watchdog_set_prescale(int prescale) {
    WDOG_PRESC = prescale;
}

// ensure interrupts remain on
void watchdog_feed() {
    mock_noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    mock_interrupts();
}
