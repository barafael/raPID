#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "Arduino.h"

void init_watchdog();
void watchdog_feed();
void watchdog_set_prescale(int prescale);

#endif // WATCHDOG_H
