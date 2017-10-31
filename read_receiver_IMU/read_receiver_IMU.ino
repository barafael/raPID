#include "error_handling.h"
#include "types.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/* HARDWARE SETUP

     MPU6050 Breakout ----- Teensy 3.2
     3.3V ----------------- 3.3V
     GND ------------------ GND
     SDA ------------------ A4/pin 18
     SCL ------------------ A5/pin 19
     INT ------------------ Digital Pin 2
*/

/* Class default I2C address is 0x68
   specific I2C addresses may be passed as a parameter here
   AD0 low = 0x68
   (default for SparkFun breakout and InvenSense evaluation board)
   AD0 high = 0x69 */
MPU6050 mpu;

static const uint8_t LED_PIN = 13;
bool blinkState = false;

/* MPU control/status vars */

/* Holds actual interrupt status byte from MPU */
uint8_t mpuIntStatus;  

/* Return status after each device operation
   (0 = success, !0 = error) */
uint8_t devStatus;

/* Expected DMP packet size (default is 42 bytes) */
uint16_t packetSize;

uint16_t fifoCount;
uint8_t fifoBuffer[64];

/* Orientation/motion vars */
Quaternion q;   // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]        gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]       world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]       gravity vector

float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ————————————————————————————————————————————————————————————————
// ———             MPU INTERRUPT DETECTION ROUTINE              ———
// ————————————————————————————————————————————————————————————————

/* Indicates whether MPU interrupt pin has gone high */
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// ————————————————————————————————————————————————————————————————
// ———             RECEIVER READ GLOBAL VARIABLES               ———
// ————————————————————————————————————————————————————————————————

static volatile byte input_flags;

/* The servo interrupt writes to this variable and the main loop reads */
volatile uint16_t receiverInShared[NUM_CHANNELS];

uint16_t receiverIn[NUM_CHANNELS];

/* Written by interrupt when HIGH value is read */
uint32_t receiverInStart[NUM_CHANNELS];

/* Read each new value, indicated by the corresponding bit set in input_flags */
void readReceiver() {
    noInterrupts();
    for (size_t channel = 0; channel < NUM_CHANNELS; channel++) {
        if (input_flags & (1 << channel)) {
            receiverIn[channel] = receiverInShared[channel];
        }
    }
    interrupts();
}

// ————————————————————————————————————————————————————
// ———             SERIAL DEBUG OUTPUT              ———
// ————————————————————————————————————————————————————

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

// ———————————————————————————————————————————————————
// ———             IMU INITIALISATION              ———
// ———————————————————————————————————————————————————

void initMPU6050() {
    /* Join I2C bus (I2Cdev library doesn't do this automatically) */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    /* 400kHz I2C clock (200kHz if CPU is 8MHz) */
    TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    /* Initialize device */
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    /* Verify connection */
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
            : F("MPU6050 connection failed"));

    /* Load and configure the DMP */
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    /* Supply your own gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    /* Make sure initialisation worked (returns 0 if so) */
    if (devStatus == 0) {
        /* Turn on the DMP, now that it's ready */
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        /* Enable Arduino interrupt detection */
        Serial.println(
                F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(2, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        /* Get expected DMP packet size for later comparison */
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        /* Error while init */
        switch (devStatus) {
            case 1:
                error_blink(DMP_INIT_MEM_LOAD_FAILED,
                        "DMP init error code 1: Initial Memory Load failed!");
                break;
            case 2:
                error_blink(DMP_CONF_UPDATES_FAILED,
                        "DMP init error code 2: DMP configuration updates failed!");
                break;
            default: {
                         char msg[50] = "DMP init error code     ";
                         itoa(devStatus, msg + 20, 10);
                         error_blink(DMP_ERROR_UNKNOWN, msg);
                         break;
                     }
        }
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(9600);

    initMPU6050();

    /* The pinMode should be correct by default, set it anyway */
    pinMode(THROTTLE_INPUT_PIN, INPUT);
    pinMode(ROLL_INPUT_PIN,     INPUT);
    pinMode(PITCH_INPUT_PIN,    INPUT);
    pinMode(YAW_INPUT_PIN,      INPUT);

    /* On each CHANGE on an input pin, an interrupt handler is called */
    attachInterrupt(THROTTLE_INPUT_PIN, readThrottle, CHANGE);
    attachInterrupt(ROLL_INPUT_PIN,     readRoll,     CHANGE);
    attachInterrupt(PITCH_INPUT_PIN,    readPitch,    CHANGE);
    attachInterrupt(YAW_INPUT_PIN,      readYaw,      CHANGE);
}

void loop() {
    /* wait for MPU interrupt or extra packet(s) available */
    while (!mpuInterrupt && fifoCount < packetSize) {
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data

        readReceiver();
        // printReceivers();
        printYPR();
    }

    /* Reset interrupt flag and get INT_STATUS byte */
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    /* Get current FIFO count */
    fifoCount = mpu.getFIFOCount();

    /* Check for overflow (this should be rare) */
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        /* reset so we can continue cleanly */
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        /* Otherwise, check for DMP data ready interrupt (this happens often) */
    } else if (mpuIntStatus & 0x02) {
        /* Wait for correct available data length, should be a VERY short wait */
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        /* Read a packet from FIFO */
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        /* Track FIFO count here in case there is > 1 packet available */
        /* (this lets us immediately read more without waiting for an interrupt) */
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        /* Yaw degrees */
        // Add M_PI to get positive values (ypr[0] element of (-M_PI, M_PI)).
        // Angle in degree is ratio of reading to max reading * 180
        // where max reading: 2 * M_PI
        int yaw_value = (int)180 - (ypr[0] + M_PI) * 180 / (M_PI * 2);
        // yaw_value = yaw_value > 180.0 ? 180.0 : yaw_value;
        // yaw_value = yaw_value < 0.0 ? 0.0 : yaw_value;

        /* Pitch degrees */
        // Add 90 to start at horizontal, flat position
        // Angle in degree is ratio of reading to max reading * 180
        // where max reading: 2 * M_PI
        int pitch_value = (int)(90 + ypr[1] * 180 / M_PI);

        /* Blink LED to indicate activity */
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

// ————————————————————————————————————————————————————————————————
// ———             RECEIVER READ INTERRUPT ROUTINES             ———
// ————————————————————————————————————————————————————————————————

void readThrottle() {
    if (digitalRead(THROTTLE_INPUT_PIN) == HIGH) {
        receiverInStart[THROTTLE_CHANNEL] = micros();
    } else {
        receiverInShared[THROTTLE_CHANNEL] =
            (uint16_t)(micros() - receiverInStart[THROTTLE_CHANNEL]);
        input_flags |= 1 << THROTTLE_CHANNEL;
    }
}

void readRoll() {
    if (digitalRead(ROLL_INPUT_PIN) == HIGH) {
        receiverInStart[ROLL_CHANNEL] = micros();
    } else {
        receiverInShared[ROLL_CHANNEL] =
            (uint16_t)(micros() - receiverInStart[ROLL_CHANNEL]);
        input_flags |= 1 << ROLL_CHANNEL;
    }
}

void readPitch() {
    if (digitalRead(PITCH_INPUT_PIN) == HIGH) {
        receiverInStart[PITCH_CHANNEL] = micros();
    } else {
        receiverInShared[PITCH_CHANNEL] =
            (uint16_t)(micros() - receiverInStart[PITCH_CHANNEL]);
        input_flags |= 1 << PITCH_CHANNEL;
    }
}

void readYaw() {
    if (digitalRead(YAW_INPUT_PIN) == HIGH) {
        receiverInStart[YAW_CHANNEL] = micros();
    } else {
        receiverInShared[YAW_CHANNEL] =
            (uint16_t)(micros() - receiverInStart[YAW_CHANNEL]);
        input_flags |= 1 << YAW_CHANNEL;
    }
}

