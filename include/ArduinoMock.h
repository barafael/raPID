#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <stdint.h>
#include <stddef.h>

#define M_PI_F 3.14159265358979323846f

int64_t mock_millis(void);

int64_t mock_micros(void);

#define INTERRUPTS_OFF 0
#define INTERRUPTS_ON  1

// Interrupts are on at boot
//@ ghost int interrupt_status = INTERRUPTS_ON;

void mock_interrupts();
void mock_noInterrupts();

void mock_digitalWrite(int pin, int state);
int mock_digitalRead(int pin);

void mock_analogWrite(int pin, int state);
int mock_analogRead(int pin);

void mock_delay(int millis);
void mock_delayMicroseconds(uint64_t micros);
void mock_pinMode(int pin, int mode);

#define HIGH 1
#define LOW 0

#define OUTPUT 0
#define INPUT 1

#define CHANGE 0
void attachInterrupt(int pin, void f(), int mode);

#endif // ARDUINO_MOCK_H
