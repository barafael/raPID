#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Teensy's digital pin 2.

   HARDWARE SETUP:
     MPU6050 Breakout ----- Teensy 3.2
     3.3V ----------------- 3.3V
     GND ------------------ GND
     SDA ------------------ A4/pin 18
     SCL ------------------ A5/pin 19
     INT ------------------ Digital Pin 2 (Teensy)
   ========================================================================= */

static const uint8_t LED_PIN = 13;
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

typedef enum {
    YAW_ANGLE = 0,
    PITCHT_ANGLE = 1,
    ROLL_ANGLE = 2
} sensor_output;

const static uint8_t NUM_CHANNELS = 4;

/* On an Arduino Uno, digital pins 2 and 3 are capable of interrupts. */
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

static volatile byte input_flags;

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

void printYPR() {
    Serial.print("ypr\t");
    Serial.print(ypr[YAW_ANGLE]);
    Serial.print("\t");
    Serial.print(ypr[PITCH_ANGLE]);
    Serial.print("\t");
    Serial.println(ypr[ROLL_ANGLE]);
}

void printReceivers() {
    Serial.print(receiverIn[THROTTLE_CHANNEL]);
    Serial.print('\t');
    Serial.print(receiverIn[ROLL_CHANNEL]);
    Serial.print('\t');
    Serial.print(receiverIn[PITCH_CHANNEL]);
    Serial.print('\t');
    Serial.print(receiverIn[YAW_CHANNEL]);
    Serial.println('\t');
}

void initMPU6050() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(2, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));

        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(500);
            digitalWrite(LED_PIN, LOW);
            delay(500);
        }
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(9600);

    initMPU6050();

    /* On each CHANGE on an input pin, the corresponding interrupt handler is called
       Note that on Arduino all digital pins default to input mode */
    attachInterrupt(THROTTLE_INPUT_PIN, readThrottle, CHANGE);
    attachInterrupt(ROLL_INPUT_PIN,     readRoll,     CHANGE);
    attachInterrupt(PITCH_INPUT_PIN,    readPitch,    CHANGE);
    attachInterrupt(YAW_INPUT_PIN,      readYaw,      CHANGE);
}

void loop() {
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .

        readReceiver();
        printReceivers();
        //printYPR();
    }
    // reset interrupt flag and get INT_STATUS byte

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // yaw degrees
        // Add M_PI to get positive values (ypr[0] element of (-M_PI, M_PI)).
        // Angle in degree is ratio of reading to max reading * 180 where max reading: 2 * M_PI
        int yaw_value = (int)180 - (ypr[0] + M_PI) * 180 / (M_PI * 2);
        /*yaw_value = yaw_value > 180.0 ? 180.0 : yaw_value;
          yaw_value = yaw_value < 0.0 ? 0.0 : yaw_value;*/
        // pitch degrees
        // Add 90 to start at horizontal, flat position
        // Angle in degree is ratio of reading to max reading * 180 where max reading: 2 * M_PI
        int pitch_value = (int) (90 + ypr[1] * 180 / M_PI);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void readThrottle() {
    if (digitalRead(THROTTLE_INPUT_PIN) == HIGH) {
        receiverInStart[THROTTLE_CHANNEL] = micros();
    } else {
        receiverInShared[THROTTLE_CHANNEL] = (uint16_t)(micros() - receiverInStart[THROTTLE_CHANNEL]);
        input_flags |= 1 << THROTTLE_CHANNEL;
    }
}

void readRoll() {
    if (digitalRead(ROLL_INPUT_PIN) == HIGH) {
        receiverInStart[ROLL_CHANNEL] = micros();
    } else {
        receiverInShared[ROLL_CHANNEL] = (uint16_t)(micros() - receiverInStart[ROLL_CHANNEL]);
        input_flags |= 1 << ROLL_CHANNEL;
    }
}

void readPitch() {
    if (digitalRead(PITCH_INPUT_PIN) == HIGH) {
        receiverInStart[PITCH_CHANNEL] = micros();
    } else {
        receiverInShared[PITCH_CHANNEL] = (uint16_t)(micros() - receiverInStart[PITCH_CHANNEL]);
        input_flags |= 1 << PITCH_CHANNEL;
    }
}

void readYaw() {
    if (digitalRead(YAW_INPUT_PIN) == HIGH) {
        receiverInStart[YAW_CHANNEL] = micros();
    } else {
        receiverInShared[YAW_CHANNEL] = (uint16_t)(micros() - receiverInStart[YAW_CHANNEL]);
        input_flags |= 1 << YAW_CHANNEL;
    }
}

