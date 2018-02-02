#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "Arduino.h"

class Watchdog {
    public:
        Watchdog();
        void feed();
};

#endif // WATCHDOG_H
