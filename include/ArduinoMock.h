#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#define FRAMAC

#ifndef FRAMAC
#include "Arduino.h"

#else


#define nullptr ((void*)0)

long mock_millis(void);

long mock_micros(void);

#endif
#endif // ARDUINO_MOCK_H
