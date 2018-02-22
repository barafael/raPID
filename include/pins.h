#ifndef PINS_H
#define PINS_H

#include <stdint.h>

static const uint8_t DEBUG_PIN = 15;

static const uint8_t LED_PIN = 15;

static const uint8_t SENTRAL_GND = 13;
static const uint8_t SENTRAL_POWER = 14;

static const uint8_t SENTRAL_INTERRUPT_PIN = 8;

typedef enum {
    THROTTLE_INPUT_PIN  = 23,
    ROLL_INPUT_PIN      = 22,
    PITCH_INPUT_PIN     = 21,
    YAW_INPUT_PIN       = 20,
    AUX1_INPUT_PIN      = 19,
    AUX2_INPUT_PIN      = 18
} input_pin;

typedef enum {
    LEFT_SERVO_PIN  = 5,
    RIGHT_SERVO_PIN = 4,
    FRONT_SERVO_PIN = 3,
    BACK_SERVO_PIN  = 2
} output_pin;

#endif // PINS_H
