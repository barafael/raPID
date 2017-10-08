/* On an Arduino Uno, digital pins 2 and 3 are capable of interrupts. */
#define RECEIVER_INPUT_PIN 2

/* The interrupt writes to this variable and the main program reads */
volatile uint16_t receiverInShared;

static uint16_t receiverIn = 0;

/* Keep old input to check if input has changed */
static uint16_t oldInput = 1;

/* Written by interrupt when HIGH value is read */
uint32_t receiverInStart;

void setup() {
    Serial.begin(9600);

    pinMode(RECEIVER_INPUT_PIN, INPUT);

    /* On each CHANGE on pin RECEIVER_INPUT_PIN, the function readReceiver will be called */
    attachInterrupt(digitalPinToInterrupt(RECEIVER_INPUT_PIN), readReceiver, CHANGE);
}

void loop() {
    /* Disable interrupts while reading shared data */
    noInterrupts();
    receiverIn = receiverInShared;
    interrupts();

    /* Check if there is a new value */
    if ((oldInput != receiverIn)) {
        Serial.println(receiverIn);

        oldInput = receiverIn;
    }

}

void readReceiver() {
    if (digitalRead(RECEIVER_INPUT_PIN) == HIGH) {
        receiverInStart = micros();
    } else {
        receiverInShared = (uint16_t)(micros() - receiverInStart);
    }
}
