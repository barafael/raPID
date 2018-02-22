#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "Arduino.h"

class Watchdog {
    public:
        Watchdog(int prescale = 4);
        void feed();
};

#endif // WATCHDOG_H
