#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "wdog_fake_registers.h"

void init_watchdog();
void watchdog_feed();
void watchdog_set_prescale(int prescale);

#endif // WATCHDOG_H
