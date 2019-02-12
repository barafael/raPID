#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "./wdog_registers.h"
#include "./ArduinoMock.h"

void init_watchdog();
void watchdog_feed();

#endif // WATCHDOG_H
