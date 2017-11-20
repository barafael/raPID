#ifndef PINS_H
#define PINS_H

#define DEBUG_PIN 23

typedef enum {
    THROTTLE_INPUT_PIN = 12,
    ROLL_INPUT_PIN     = 11,
    PITCH_INPUT_PIN    = 10,
    YAW_INPUT_PIN      = 9,
} input_pin;

typedef enum {
    LEFT_SERVO_PIN = 21;
    RIGHT_SERVO_PIN = 22;
} output_pin;

#endif // PINS_H
