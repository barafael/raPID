#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#define I2C_MASTER 0
#define I2C_PINS_16_17 0
#define I2C_PULLUP_EXT 0
#define I2C_RATE_400 0

#define I2C_NOSTOP 0
#define M_PI_F 3.14159265358979323846f
#define PI M_PI_F

uint64_t milliseconds = 1;
uint64_t microseconds = 1;

uint64_t mock_millis(void);

uint64_t mock_micros(void);

#define INTERRUPTS_OFF 0
#define INTERRUPTS_ON  1

// Interrupts are on at boot
//@ ghost int ghost_interrupt_status = INTERRUPTS_ON;

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
#define RISING 1
#define FALLING 2
void mock_attachInterrupt(int pin, void f(), int mode);

#endif // ARDUINO_MOCK_H
