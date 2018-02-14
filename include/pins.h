#ifndef PINS_H
#define PINS_H

#include <stdint.h>

static const uint8_t DEBUG_PIN = 15;

static const uint8_t LED_PIN = 13;
static const uint8_t ARMING_STATE_PIN = 14;

static const uint8_t MPU_INTERRUPT_PIN = 12;

typedef enum {
    THROTTLE_INPUT_PIN  = 11,
    ROLL_INPUT_PIN      = 10,
    PITCH_INPUT_PIN     = 9,
    YAW_INPUT_PIN       = 8,
    AUX1_INPUT_PIN      = 7,
    AUX2_INPUT_PIN      = 6
} input_pin;

typedef enum {
    LEFT_SERVO_PIN  = 5,
    RIGHT_SERVO_PIN = 4,
    FRONT_SERVO_PIN = 3,
    BACK_SERVO_PIN  = 2
} output_pin;

#endif // PINS_H
