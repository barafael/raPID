#include <Servo.h>

const static uint8_t NUM_CHANNELS = 4;

/* On an Arduino Uno, digital pins 2 and 3 are capable of interrupts. */
typedef enum {
    THROTTLE_INPUT_PIN = 12,
    ROLL_INPUT_PIN     = 11,
    PITCH_INPUT_PIN    = 10,
    YAW_INPUT_PIN      = 9,
} input_pin;

typedef enum {
    THROTTLE = 0,
    ROLL     = 1,
    PITCH    = 2,
    YAW      = 3
} input_channel;

static volatile byte input_flags;

Servo throttle, roll;

/* The interrupt writes to this variable and the main program reads */
volatile uint16_t receiverInShared[NUM_CHANNELS];

static uint16_t receiverIn[NUM_CHANNELS];

/* Written by interrupt when HIGH value is read */
uint32_t receiverInStart[NUM_CHANNELS];

void readReceiver() {
    noInterrupts();
    for (size_t channel = 0; channel < NUM_CHANNELS; channel++) {
        if (input_flags & (1 << channel)) {
            receiverIn[channel] = receiverInShared[channel];
        }
    }
    interrupts();
}

void printReceivers() {
    Serial.print(receiverIn[THROTTLE]);
    Serial.print('\t');
    Serial.print(receiverIn[ROLL]);
    Serial.print('\t');
    Serial.print(receiverIn[PITCH]);
    Serial.print('\t');
    Serial.print(receiverIn[YAW]);
    Serial.println('\t');
}

void setup() {
    throttle.attach(20);
    roll.attach(21);

    Serial.begin(9600);

    /* On each CHANGE on an input pin, the corresponding interrupt handler is called
       Note that on Arduino all digital pins default to input mode */
    attachInterrupt(THROTTLE_INPUT_PIN, readThrottle, CHANGE);
    attachInterrupt(ROLL_INPUT_PIN,     readRoll,     CHANGE);
    attachInterrupt(PITCH_INPUT_PIN,    readPitch,    CHANGE);
    attachInterrupt(YAW_INPUT_PIN,      readYaw,      CHANGE);
}

void loop() {
    readReceiver();
    printReceivers();
}

void readThrottle() {
    if (digitalRead(THROTTLE_INPUT_PIN) == HIGH) {
        receiverInStart[THROTTLE] = micros();
    } else {
        receiverInShared[THROTTLE] = (uint16_t)(micros() - receiverInStart[THROTTLE]);
        input_flags |= 1 << THROTTLE;
    }
}

void readRoll() {
    if (digitalRead(ROLL_INPUT_PIN) == HIGH) {
        receiverInStart[ROLL] = micros();
    } else {
        receiverInShared[ROLL] = (uint16_t)(micros() - receiverInStart[ROLL]);
        input_flags |= 1 << ROLL;
    }
}

void readPitch() {
    if (digitalRead(PITCH_INPUT_PIN) == HIGH) {
        receiverInStart[PITCH] = micros();
    } else {
        receiverInShared[PITCH] = (uint16_t)(micros() - receiverInStart[PITCH]);
        input_flags |= 1 << PITCH;
    }
}

void readYaw() {
    if (digitalRead(YAW_INPUT_PIN) == HIGH) {
        receiverInStart[YAW] = micros();
    } else {
        receiverInShared[YAW] = (uint16_t)(micros() - receiverInStart[YAW]);
        input_flags |= 1 << YAW;
    }
}

