#ifndef PINS_H
#define PINS_H

static const uint8_t DEBUG_PIN = 15;

static const uint8_t LED_PIN = 14;

static const uint8_t MPU_INTERRUPT_PIN = 13;

typedef enum {
    THROTTLE_INPUT_PIN  = 12,
    ROLL_INPUT_PIN      = 11,
    PITCH_INPUT_PIN     = 10,
    YAW_INPUT_PIN       = 9,
    AUX1_INPUT_PIN      = 8,
    AUX2_INPUT_PIN      = 7
} input_pin;

typedef enum {
    LEFT_SERVO_PIN  = 6,
    RIGHT_SERVO_PIN = 5,
    FRONT_SERVO_PIN = 4,
    BACK_SERVO_PIN  = 3
} output_pin;

#endif // PINS_H
