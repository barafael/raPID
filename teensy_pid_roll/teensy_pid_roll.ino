/*
   ——————————————————————————————————————————————
   ———             HARDWARE SETUP             ———
   ——————————————————————————————————————————————

   MPU6050 Breakout ----- Teensy 3.2
   3.3V ----------------- 3.3V
   GND ------------------ GND
   SDA ------------------ A4/pin 18
   SCL ------------------ A5/pin 19
   INT ------------------ Digital Pin 2

   PPM from RC RX go to pins 9, 10, 11, 12 (see types.h)
   Output PPM to ESC's: pins 20, 21
*/

#include "error_handling.h"
#include "types.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "Servo.h"


//  —————————————————————————————————————————————————
//  ———              MPU6050 VARIABLES            ———
//  —————————————————————————————————————————————————

/* Class default I2C address is 0x68
   specific I2C addresses may be passed as a parameter here
   AD0 low = 0x68
   (default for SparkFun breakout and InvenSense evaluation board)
   AD0 high = 0x69 */
MPU6050 mpu;

static const uint8_t mpu_address = 0x68;

static const uint8_t LED_PIN = 13;
bool blinkState = false;

/* Holds actual interrupt status byte from MPU */
uint8_t mpuIntStatus;

/* Return status after each device operation
   (0 = success, !0 = error) */
uint8_t devStatus;

/* Expected DMP packet size (default is 42 bytes) */
uint16_t packetSize;

uint16_t fifoCount;
uint8_t fifoBuffer[64];


//  —————————————————————————————————————————————————
//  ———          ORIENTATION/MOTION VARS          ———
//  —————————————————————————————————————————————————

Quaternion q;        // [w, x, y, z]    quaternion container
VectorInt16 aa;      // [x, y, z]       accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]       gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]       world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]       gravity vector

float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/* Scaled ypr to [0, 1000] */
uint16_t attitude[3];

/* Angular Rates */
int16_t gyro_axis[3] = { 0 };
int64_t gyro_axis_cal[3] = { 0 };


// ————————————————————————————————————————————————————————————————
// ———             MPU INTERRUPT DETECTION ROUTINE              ———
// ————————————————————————————————————————————————————————————————

/* Indicates whether MPU interrupt pin has gone high */
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
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
        default:
            {
                char msg[50] = "DMP init error code     ";
                itoa(devStatus, msg + 20, 10);
                error_blink(DMP_ERROR_UNKNOWN, msg);
                break;
            }
        }
    }
}


// ————————————————————————————————————————————————————————————————
// ———             CALIBRATE RATES BY TAKING AVG                ———
// ————————————————————————————————————————————————————————————————

static bool rate_calibrated = false;

void calib_rates() {
    uint16_t iterations = 300;

    Serial.println(F("Calibrating gyro rates, hold still!"));

    int16_t raw_rates[3] = { 0 };

    while(!calib_rates_ok()) {
        for (int i = 0; i < 3; i++) {
            gyro_axis_cal[i] = 0;
        }
        for (uint16_t count = 0; count < iterations; count++) {
            read_raw_rates(raw_rates);
            gyro_axis_cal[ROLL_RATE]  += raw_rates[0];
            gyro_axis_cal[PITCH_RATE] += raw_rates[1];
            gyro_axis_cal[YAW_RATE]   += raw_rates[2];

            delay(5);
        }

        gyro_axis_cal[ROLL_RATE]  /= iterations;
        gyro_axis_cal[PITCH_RATE] /= iterations;
        gyro_axis_cal[YAW_RATE]   /= iterations;
    }

    Serial.println(F("Done calibrating gyro rates"));
}

bool calib_rates_ok() {
    const int iterations = 50;
    const int tolerance = 10;

    int64_t accumulator[3] = { 0 };
    
    for (uint16_t count = 0; count < iterations; count++) {
        read_angular_rates();
        accumulator[ROLL_RATE]  += gyro_axis[ROLL_RATE];
        accumulator[PITCH_RATE] += gyro_axis[PITCH_RATE];
        accumulator[YAW_RATE]   += gyro_axis[YAW_RATE];

        delay(5);
    }

    accumulator[ROLL_RATE]  /= iterations;
    accumulator[PITCH_RATE] /= iterations;
    accumulator[YAW_RATE]   /= iterations;

    Serial.print("Average rate over ");
    Serial.print(iterations);
    Serial.println(" iterations: ");
    Serial.print((uint32_t)accumulator[ROLL_RATE]);
    Serial.print("\t");
    Serial.print((uint32_t)accumulator[PITCH_RATE]);
    Serial.print("\t");
    Serial.println((uint32_t)accumulator[YAW_RATE]);

    rate_calibrated =
           (abs(accumulator[ROLL_RATE])  < tolerance) &&
           (abs(accumulator[PITCH_RATE]) < tolerance) &&
           (abs(accumulator[YAW_RATE])   < tolerance);
    return rate_calibrated;
}


// ————————————————————————————————————————————————————————————————
// ———             FETCH ANGULAR RATES FROM IMU                 ———
// ————————————————————————————————————————————————————————————————

void read_raw_rates(int16_t *rates) {
    Wire.beginTransmission(mpu_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(mpu_address, 6);
    while (Wire.available() < 6);
    rates[ROLL_RATE]  = Wire.read() << 8 | Wire.read();
    rates[PITCH_RATE] = Wire.read() << 8 | Wire.read();
    rates[YAW_RATE]   = Wire.read() << 8 | Wire.read();
}

void read_angular_rates() {
    Wire.beginTransmission(mpu_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(mpu_address, 6);
    while (Wire.available() < 6);
    gyro_axis[ROLL_RATE]  = Wire.read() << 8 | Wire.read();
    gyro_axis[PITCH_RATE] = Wire.read() << 8 | Wire.read();
    gyro_axis[YAW_RATE]   = Wire.read() << 8 | Wire.read();

    gyro_axis[ROLL_RATE]  -= gyro_axis_cal[ROLL_RATE];
    gyro_axis[PITCH_RATE] -= gyro_axis_cal[PITCH_RATE];
    gyro_axis[YAW_RATE]   -= gyro_axis_cal[YAW_RATE];
}


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
// ———           SERVO GLOBAL VARIABLES             ———
// ————————————————————————————————————————————————————

static const uint8_t LEFT_SERVO_PIN = 21;
static const uint8_t RIGHT_SERVO_PIN = 22;

Servo left;
Servo right;
uint16_t left_throttle;
uint16_t right_throttle;
uint16_t throttle;

/* Arm ESC's with a long low pulse */

void armESC() {
    Serial.println("Initialising ESCs: 1000ms pulse");
    left.writeMicroseconds(1000);
    right.writeMicroseconds(1000);
    delay(1500);
    Serial.println("Initialised ESCs");
}


// ————————————————————————————————————————————————————
// ———        PID VARIABLES AND COEFFICIENTS        ———
// ————————————————————————————————————————————————————

uint16_t last_roll;

float pid_output_roll;

float pid_p_gain_roll = 3.0;
float pid_i_gain_roll = 0.0;
float pid_d_gain_roll = 0.0;
int pid_max_roll = 400;

float pid_error;
float pid_last_error;

float pid_i_mem_roll;
float pid_roll_setpoint;

/* Calculate PID output based on absolute angle in attitude[] */
void calculatePID_absolute() {
    pid_error = attitude[ROLL_ANGLE] - receiverIn[ROLL_CHANNEL] + 1000;
    Serial.println(pid_error);

    float p = pid_p_gain_roll * pid_error;
    pid_i_mem_roll += (pid_i_gain_roll * pid_error);
    if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
    else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = (pid_max_roll * -1);

    pid_output_roll = p + pid_i_mem_roll + (pid_d_gain_roll * (pid_error - pid_last_error));
    if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

    pid_last_error = pid_error;
}

/* Calculate PID output based on angular rate */
void calculatePID_angular_rate() {
    pid_error = attitude[ROLL_ANGLE] - receiverIn[ROLL_CHANNEL] + 1000;
    Serial.println(pid_error);

    float p = pid_p_gain_roll * pid_error;
    pid_i_mem_roll += (pid_i_gain_roll * pid_error);
    if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
    else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = (pid_max_roll * -1);

    pid_output_roll = p + pid_i_mem_roll + (pid_d_gain_roll * (pid_error - pid_last_error));
    if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

    pid_last_error = pid_error;
}


// ————————————————————————————————————————————————————
// ———             SERIAL DEBUG OUTPUT              ———
// ————————————————————————————————————————————————————

void printYPR() {
    Serial.print(F("ypr\t"));
    Serial.print(ypr[YAW_ANGLE]);
    Serial.print(F("\t"));
    Serial.print(ypr[PITCH_ANGLE]);
    Serial.print(F("\t"));
    Serial.println(ypr[ROLL_ANGLE]);
}

void printAttitude() {
    Serial.print(F("Attitude\t"));
    Serial.print(attitude[YAW_ANGLE]);
    Serial.print(F("\t"));
    Serial.print(attitude[PITCH_ANGLE]);
    Serial.print(F("\t"));
    Serial.println(attitude[ROLL_ANGLE]);
}

void printReceivers() {
    Serial.print(receiverIn[THROTTLE_CHANNEL]);
    Serial.print(F("\t"));
    Serial.print(receiverIn[ROLL_CHANNEL]);
    Serial.print(F("\t"));
    Serial.print(receiverIn[PITCH_CHANNEL]);
    Serial.print(F("\t"));
    Serial.println(receiverIn[YAW_CHANNEL]);
}

void printAngular() {
    Serial.print(gyro_axis[ROLL_RATE]);
    Serial.print("\t");
    Serial.print(gyro_axis[PITCH_RATE]);
    Serial.print("\t");
    Serial.println(gyro_axis[YAW_RATE]);
}

void print_binary(int value, int num_places) {
    int mask = 0;

    for (int n = 1; n <= num_places; n++) {
        mask = (mask << 1) | 0x0001;
    }
    value = value & mask; // truncate v to specified number of places

    while (num_places) {

        if (value & (0x0001 << num_places - 1)) {
            Serial.print(F("1"));
        } else {
            Serial.print(F("0"));
        }

        --num_places;
        if (((num_places % 4) == 0) && (num_places != 0)) {
            Serial.print("_");
        }
    }
    Serial.println();
}


void setup() {
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(9600);

    left.attach(LEFT_SERVO_PIN);
    right.attach(RIGHT_SERVO_PIN);

    armESC();

    initMPU6050();

    calib_rates();

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
    readReceiver();
    printAngular();
    // printReceivers();
    // printAttitude();
    // printYPR();
    // calculatePID();

    /*
       throttle = receiverIn[THROTTLE_CHANNEL];
       throttle = throttle > 1800 ? 1800 : throttle;

       left_throttle  = throttle + pid_output_roll;
       right_throttle = throttle + pid_output_roll;

       left_throttle = left_throttle < 1000 ? 1000 : left_throttle;
       right_throttle = right_throttle < 1000 ? 1000 : right_throttle;

       left.writeMicroseconds(left_throttle);
       right.writeMicroseconds(right_throttle);

       Serial.print(pid_output_roll);
       Serial.print("\t");
       Serial.print(left_throttle);
       Serial.print("\t");
       Serial.println(right_throttle);
     */

    /* wait for MPU interrupt or extra packet(s) available */
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    if(mpuInterrupt || fifoCount >= packetSize) {
        readMPU();
    }

    /* Blink LED to indicate activity */
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

void readMPU() {
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

            read_angular_rates();

            //mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            /* Yaw degrees */
            // Add M_PI to get positive values (ypr[0] element of (-M_PI, M_PI)).
            // Angle in degree is ratio of reading to max reading * 180
            // where max reading: 2 * M_PI
            // int yaw_value = (int)180 - (ypr[0] + M_PI) * 180 / (M_PI * 2);
            // yaw_value = yaw_value > 180.0 ? 180.0 : yaw_value;
            // yaw_value = yaw_value < 0.0 ? 0.0 : yaw_value;

            /* Pitch degrees */
            // Add 90 to start at horizontal, flat position
            // Angle in degree is ratio of reading to max reading * 180
            // where max reading: 2 * M_PI
            // int pitch_value = (int)(90 + ypr[1] * 180 / M_PI);

            for (size_t index = YAW_ANGLE; index <= ROLL_ANGLE; index++) {
                attitude[index] = (ypr[index] + M_PI) * (1000 / (2 * M_PI));
            }
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
