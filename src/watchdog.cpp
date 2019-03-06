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
    assigns ghost_interrupt_status;

    // actually, these are volatile registers and not memory
    //ensures WDOG_UNLOCK == WDOG_UNLOCK_SEQ2;
    //ensures WDOG_PRESC == 4;

    ensures GLOBALinterruptReenable: ghost_interrupt_status == INTERRUPTS_ON;
*/
void init_watchdog() {
    mock_noInterrupts();

    //@ assert GLOBALinterruptSafety: INTERRUPTS_OFF;
    // unlock access to WDOG registers
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    mock_delayMicroseconds(1);

    // 1 second WDT timeout (e.g. reset in < 1 sec or a reset occurs)
    WDOG_TOVALH = 0x006d;
    WDOG_TOVALL = 0xdd00;

    // watchdog timer at 7.2MHz
    WDOG_PRESC  = 0x400;

    // Enable WDT.
    // This must happen in one single write to WDOG_CTRLH
    WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
        WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
        WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
    mock_interrupts();
}

/*@ requires \valid(&WDOG_REFRESH);
    assigns WDOG_REFRESH;
    assigns ghost_interrupt_status;
    ensures GLOBALinterruptReenable: ghost_interrupt_status == INTERRUPTS_ON;
*/
void watchdog_feed() {
    mock_noInterrupts();
    //@assert GLOBALinterruptSafety: ghost_interrupt_status == INTERRUPTS_OFF;
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    mock_interrupts();
    //@assert GLOBALinterruptReenable: ghost_interrupt_status == INTERRUPTS_ON;
}
