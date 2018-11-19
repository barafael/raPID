#ifndef WDOG_REGS
#define WDOG_REGS

#include <stdint.h>
#define WDOG_UNLOCK      (*(volatile uint16_t *)0x4005200E) // Watchdog Unlock register
#define WDOG_UNLOCK_SEQ1 ((uint16_t)0xC520)
#define WDOG_UNLOCK_SEQ2 ((uint16_t)0xD928)
#define WDOG_STCTRLH     (*(volatile uint16_t *)0x40052000) // Watchdog Status and Control Register High

#define WDOG_TOVALL      (*(volatile uint16_t *)0x40052006) // Watchdog Time-out Value Register Low
#define WDOG_TOVALH      (*(volatile uint16_t *)0x40052004) // Watchdog Time-out Value Register High
#define WDOG_PRESC       (*(volatile uint16_t *)0x40052016) // Watchdog Prescaler register
#define WDOG_REFRESH     (*(volatile uint16_t *)0x4005200C) // Watchdog Refresh register

#endif // WDOG_REGS
