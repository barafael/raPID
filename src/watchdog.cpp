#include "../include/watchdog.h"

/*@ requires \valid(&WDOG_UNLOCK);
    requires \valid(&WDOG_STCTRLH);
    requires \valid(&WDOG_TOVALL);
    requires \valid(&WDOG_TOVALH);
    requires \valid(&WDOG_PRESC);

    assigns WDOG_UNLOCK;
    assigns WDOG_STCTRLH;
    assigns WDOG_TOVALL;
    assigns WDOG_TOVALH;
    assigns WDOG_PRESC;

    // actually, these are volatile registers and not memory
    //ensures WDOG_UNLOCK == WDOG_UNLOCK_SEQ2;
    //ensures WDOG_PRESC == 4;
*/
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

/*@ requires \valid(&WDOG_PRESC);
    assigns WDOG_PRESC;
    // not memory, just a volatile register
    //ensures WDOG_PRESC == prescale;
*/
void watchdog_set_prescale(int prescale) {
    WDOG_PRESC = prescale;
}

/*@ requires \valid(&WDOG_REFRESH);
    assigns WDOG_REFRESH;
    assigns ghost_interrupt_status;
    ensures ghost_interrupt_status == INTERRUPTS_ON;
*/
void watchdog_feed() {
    mock_noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    mock_interrupts();
    //@assert ghost_interrupt_status == INTERRUPTS_ON;
}
