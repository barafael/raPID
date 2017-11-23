#include <Arduino.h>
#include "WProgram.h"

// TODO check out setRate from MPU6050.h
#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050_9Axis_MotionApps41.h"

#include "../include/pins.h"
#include "../include/error_handling.h"
#include "../include/settings.h"
#include "../include/serial_debug.h"


/*
   —————————————————————————————————————————————————
   ———              MPU6050 VARIABLES            ———
   —————————————————————————————————————————————————
*/

/* Class default I2C address is 0x68
   specific I2C addresses may be passed as a parameter here
   AD0 low = 0x68
   (default for SparkFun breakout and InvenSense evaluation board)
   AD0 high = 0x69
*/
static MPU6050 mpu;

static const uint8_t mpu_address = 0x68;

/* Holds actual interrupt status byte from MPU */
static uint8_t mpu_int_status;

/* Return status after each device operation
   (0 = success, !0 = error) */
static uint8_t dev_status;

/* Expected DMP packet size (default is 42 bytes) */
static uint16_t packet_size;

static uint16_t fifo_count;
static uint8_t fifo_buffer[64];

/* Indicates whether MPU interrupt pin has gone high */
static volatile bool mpu_interrupt = false;


/*
   —————————————————————————————————————————————————
   ———          ORIENTATION/MOTION VARS          ———
   —————————————————————————————————————————————————
*/

static Quaternion q;         // [w, x, y, z]    quaternion container
static VectorInt16 aa;       // [x, y, z]       accel sensor measurements
static VectorInt16 aaReal;   // [x, y, z]       gravity-free accel sensor measurements
static VectorInt16 aaWorld;  // [x, y, z]       world-frame accel sensor measurements
static VectorFloat gravity;  // [x, y, z]       gravity vector

/* Yaw/Pitch/Roll container and gravity vector
 * [yaw, pitch, roll]
 */
extern float yaw_pitch_roll[3];

/* Scaled yaw_pitch_roll to [0, 1000]
 * [yaw, pitch, roll]
 */
extern int16_t attitude[3];

/* Angular Rates
 * [yaw_rate, pitch_rate, roll_rate]
 */
extern int16_t gyro_axis[3];

/* Angular Rate calibration offsets
 * [yaw_offset, pitch_offset, roll_offset]
 */
static int64_t gyro_axis_cal[3] = { 0 };


/*
   ————————————————————————————————————————————————————————————————
   ———             FETCH ANGULAR RATES FROM IMU                 ———
   ————————————————————————————————————————————————————————————————
*/

void read_raw_rates(int16_t *rates) {
    Wire.beginTransmission(mpu_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(mpu_address, 6);
    while (Wire.available() < 6) { }
    rates[ROLL_RATE]  = Wire.read() << 8 | Wire.read();
    rates[PITCH_RATE] = Wire.read() << 8 | Wire.read();
    rates[YAW_RATE]   = Wire.read() << 8 | Wire.read();
}

void read_angular_rates() {
    Wire.beginTransmission(mpu_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(mpu_address, 6);
    while (Wire.available() < 6) { }
    gyro_axis[ROLL_RATE]  = Wire.read() << 8 | Wire.read();
    gyro_axis[PITCH_RATE] = Wire.read() << 8 | Wire.read();
    gyro_axis[YAW_RATE]   = Wire.read() << 8 | Wire.read();

    gyro_axis[ROLL_RATE]  -= gyro_axis_cal[ROLL_RATE];
    gyro_axis[PITCH_RATE] -= gyro_axis_cal[PITCH_RATE];
    gyro_axis[YAW_RATE]   -= gyro_axis_cal[YAW_RATE];
}


/*
   —————————————————————————————————————————————————————————————
   ———             FETCH ABS ANGLES FROM IMU                 ———
   —————————————————————————————————————————————————————————————
*/

void read_abs_angles() {
    /* skip if no MPU interrupt or no extra packet(s) available */
    if (!mpu_interrupt && (fifo_count < packet_size)) {
        return;
    }

    /* Reset interrupt flag and get INT_STATUS byte */
    mpu_interrupt = false;


    // 508us at TWBR = 24, 140 us at TWBR = 12
    // digitalWrite(DEBUG_PIN, HIGH);
    mpu_int_status = mpu.getIntStatus();
    // digitalWrite(DEBUG_PIN, LOW);

    /* Get current FIFO count */
    // 608us at TWBR = 24, 160 us at TWBR = 12
    // digitalWrite(DEBUG_PIN, HIGH);
    fifo_count = mpu.getFIFOCount();
    // digitalWrite(DEBUG_PIN, LOW);

    /* Check for overflow (this should be rare) */
    if ((mpu_int_status & 0x10) || fifo_count == 1024) {
        /* reset so we can continue cleanly */
        mpu.resetFIFO();
        serial_println(F("FIFO overflow!"));

        /* Otherwise, check for DMP data ready interrupt (this happens often) */
    } else if (mpu_int_status & 0x02) {
        /* Wait for correct available data length, should be a VERY short wait */
        // digitalWrite(DEBUG_PIN, HIGH);
        while (fifo_count < packet_size) {
            fifo_count = mpu.getFIFOCount();
        }
        // digitalWrite(DEBUG_PIN, LOW);

        /* Read a packet from FIFO */
        // 4.64ms at TWBR = 24, 1.28ms at TWBR = 12
        // digitalWrite(DEBUG_PIN, HIGH);
        mpu.getFIFOBytes(fifo_buffer, packet_size);
        // digitalWrite(DEBUG_PIN, LOW);

        /* Track FIFO count here in case there is > 1 packet available */
        /* (this lets us immediately read more without waiting for an interrupt) */
        fifo_count -= packet_size;

        // 230us
        // digitalWrite(DEBUG_PIN, HIGH);
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(yaw_pitch_roll, &q, &gravity);
        // digitalWrite(DEBUG_PIN, LOW);

        /* Yaw degrees */
        // Add M_PI to get positive values (yaw_pitch_roll[0] element of (-M_PI, M_PI)).
        // Angle in degree is ratio of reading to max reading * 180
        // where max reading: 2 * M_PI
        // int yaw_value = (int)180 - (yaw_pitch_roll[0] + M_PI) * 180 / (M_PI * 2);
        // yaw_value = yaw_value > 180.0 ? 180.0 : yaw_value;
        // yaw_value = yaw_value < 0.0 ? 0.0 : yaw_value;

        /* Pitch degrees */
        // Add 90 to start at horizontal, flat position
        // Angle in degree is ratio of reading to max reading * 180
        // where max reading: 2 * M_PI
        // int pitch_value = (int)(90 + yaw_pitch_roll[1] * 180 / M_PI);

        // 12.5us
        // digitalWrite(DEBUG_PIN, HIGH);
        for (size_t index = YAW_ANGLE; index <= ROLL_ANGLE; index++) {
            attitude[index] = (yaw_pitch_roll[index] + M_PI) * (1000 / (2 * M_PI));
        }
        // digitalWrite(DEBUG_PIN, LOW);
    }
}


/*
   ————————————————————————————————————————————————————————————————
   ———             CALIBRATE RATES BY TAKING AVG                ———
   ————————————————————————————————————————————————————————————————
*/

static bool rate_calibrated = false;

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

    serial_print("Average rate over ");
    serial_print(iterations);
    serial_println(" iterations: ");
    serial_print((uint32_t)accumulator[ROLL_RATE]);
    serial_print("\t");
    serial_print((uint32_t)accumulator[PITCH_RATE]);
    serial_print("\t");
    serial_println((uint32_t)accumulator[YAW_RATE]);

    rate_calibrated =
        (abs(accumulator[ROLL_RATE])  < tolerance) &&
        (abs(accumulator[PITCH_RATE]) < tolerance) &&
        (abs(accumulator[YAW_RATE])   < tolerance);

    return rate_calibrated;
}

void calib_rates() {
    uint16_t iterations = 300;

    serial_println(F("Calibrating gyro rates, hold still!"));

    int16_t raw_rates[3] = { 0 };

    /* Attempt calibration and check if it (probably) succeeded */
    while (!calib_rates_ok()) {
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

        iterations = iterations < 2000 ? iterations + 200 : iterations;
    }
}


/*
   ————————————————————————————————————————————————————————————————
   ———             MPU INTERRUPT DETECTION ROUTINE              ———
   ————————————————————————————————————————————————————————————————
*/

void dmp_data_ready() {
    mpu_interrupt = true;
}


/*
   ———————————————————————————————————————————————————
   ———             IMU INITIALISATION              ———
   ———————————————————————————————————————————————————
*/

void init_MPU6050() {
    /* Join I2C bus (I2Cdev library doesn't do this automatically) */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    /* 400kHz I2C clock (200kHz if CPU is 8MHz) */
    TWBR = 12;
    // TODO use fastwire?
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    /* Initialize device */
    serial_println(F("Initializing I2C devices..."));
    mpu.initialize();

    /* Verify connection */
    serial_println(F("Testing device connections..."));
    serial_println(mpu.testConnection() ? F("MPU6050 connection successful")
                   : F("MPU6050 connection failed"));

    /* Load and configure the DMP */
    serial_println(F("Initializing DMP..."));
    dev_status = mpu.dmpInitialize();

    /* Supply your own gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    // 1688 factory default for my test chip
    mpu.setZAccelOffset(1788);

    // TODO investigate if rate 2 has negative effect. Default is 4.
    mpu.setRate(2);
    /* Make sure initialisation worked (returns 0 if so) */
    if (dev_status == 0) {
        /* Turn on the DMP, now that it's ready */
        serial_println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        /* Enable Arduino interrupt detection */
        serial_println(
            F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        pinMode(MPU_INTERRUPT_PIN, INPUT);
        attachInterrupt(MPU_INTERRUPT_PIN, dmp_data_ready, RISING);
        mpu_int_status = mpu.getIntStatus();

        serial_println(F("DMP ready! Waiting for first interrupt..."));

        /* Get expected DMP packet size for later comparison */
        packet_size = mpu.dmpGetFIFOPacketSize();
    } else {
        /* Error while init */
        switch (dev_status) {
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
            itoa(dev_status, msg + 20, 10);
            error_blink(DMP_ERROR_UNKNOWN, msg);
            break;
        }
        }
    }
    // TODO use setOffset functions
    calib_rates();
}
