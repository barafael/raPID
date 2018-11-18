#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#ifndef FRAMAC
#include "Arduino.h"
#else

#define nullptr (0)

#define LOW 0
#define HIGH 1

#define OUTPUT 0
#define INPUT 1

#include"stdint.h"
#include"stddef.h"

uint64_t millis(void);

uint64_t micros(void);

void interrupts();
void noInterrupts();

void digitalWrite(int pin, int state);
void delay(int millis);
void pinMode(int pin, int mode);

#ifndef F
#define F(a) a
#endif
#endif
#endif // ARDUINO_MOCK_H
