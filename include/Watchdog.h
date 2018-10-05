#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "Arduino.h"

void watchdog_init();
void set_watchdog_prescale(int prescale);
void watchdog_feed();

#endif // WATCHDOG_H
