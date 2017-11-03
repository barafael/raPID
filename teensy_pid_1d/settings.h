const static uint8_t NUM_CHANNELS = 4;

typedef enum {
    YAW_ANGLE   = 0,
    PITCH_ANGLE = 1,
    ROLL_ANGLE  = 2
} angle;

typedef enum {
    ROLL_RATE  = 0,
    PITCH_RATE = 1,
    YAW_RATE   = 2
} angle_rate;

typedef enum {
    THROTTLE_INPUT_PIN = 12,
    ROLL_INPUT_PIN     = 11,
    PITCH_INPUT_PIN    = 10,
    YAW_INPUT_PIN      = 9,
} input_pin;

typedef enum {
    THROTTLE_CHANNEL = 0,
    ROLL_CHANNEL     = 1,
    PITCH_CHANNEL    = 2,
    YAW_CHANNEL      = 3
} input_channel;

