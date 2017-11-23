#ifndef PINS_H
#define PINS_H

static const uint8_t DEBUG_PIN = 23;

static const uint8_t LED_PIN = 13;

static const uint8_t MPU_INTERRUPT_PIN = 12;

typedef enum {
    THROTTLE_INPUT_PIN = 11,
    ROLL_INPUT_PIN     = 10,
    PITCH_INPUT_PIN    = 9,
    YAW_INPUT_PIN      = 8
} input_pin;

typedef enum {
    LEFT_SERVO_PIN = 21,
    RIGHT_SERVO_PIN = 22
} output_pin;

#endif // PINS_H
