#include "Arduino.h"

#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050_9Axis_MotionApps41.h"

#include "../include/error_blink.h"
#include "../include/pins.h"
#include "../include/settings.h"
#include "../include/imu.h"

static const uint16_t MPU6050_ACCEL_OFFSET_X = -1524;
static const uint16_t MPU6050_ACCEL_OFFSET_Y = -444;
static const uint16_t MPU6050_ACCEL_OFFSET_Z = 1108;
static const uint16_t MPU6050_GYRO_OFFSET_X  = 105;
static const uint16_t MPU6050_GYRO_OFFSET_Y  = 95;
static const uint16_t MPU6050_GYRO_OFFSET_Z  = -21;

/*
   -------------------------------------------------
   ---              MPU6050 VARIABLES            ---
   -------------------------------------------------
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
static uint8_t  fifo_buffer[64];

/* Indicates whether MPU interrupt pin has gone high */
static volatile bool mpu_interrupt = false;

/*
   -------------------------------------------------
   ---          ORIENTATION/MOTION VARS          ---
   -------------------------------------------------
*/

static Quaternion q;         // [w, x, y, z]    quaternion container
static VectorInt16 aa;       // [x, y, z]       accel sensor measurements
static VectorInt16 aaReal;   // [x, y, z]       gravity-free accel sensor measurements
static VectorInt16 aaWorld;  // [x, y, z]       world-frame accel sensor measurements
static VectorFloat gravity;  // [x, y, z]       gravity vector


static offset_axis_t gyro_offsets = { 0 };


/*
   ----------------------------------------------------------------
   ---      FETCH UNCALIBRATED RAW ANGULAR RATES FROM IMU       ---
   ----------------------------------------------------------------
*/

static void update_raw_rates(axis_t& raw_rates) {
    Wire.beginTransmission(mpu_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(mpu_address, 6);

    raw_rates[ROLL_AXIS]  = Wire.read() << 8 | Wire.read();
    raw_rates[PITCH_AXIS] = Wire.read() << 8 | Wire.read();
    raw_rates[YAW_AXIS]   = Wire.read() << 8 | Wire.read();
}


/*
   ----------------------------------------------------------------
   ---       FETCH ANGULAR RATES FROM IMU, CALIBRATED           ---
   ----------------------------------------------------------------
*/

void update_angular_rates(axis_t& angular_rates) {
    //digitalWrite(DEBUG_PIN, HIGH);
    Wire.beginTransmission(mpu_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(mpu_address, 6);

    angular_rates[ROLL_AXIS]  = Wire.read() << 8 | Wire.read();
    angular_rates[PITCH_AXIS] = Wire.read() << 8 | Wire.read();
    angular_rates[YAW_AXIS]   = Wire.read() << 8 | Wire.read();

    angular_rates[ROLL_AXIS]  -= gyro_offsets[ROLL_AXIS];
    angular_rates[PITCH_AXIS] -= gyro_offsets[PITCH_AXIS];
    angular_rates[YAW_AXIS]   -= gyro_offsets[YAW_AXIS];

    //digitalWrite(DEBUG_PIN, LOW);
}


/*
   -------------------------------------------------------------
   ---             FETCH ABS ANGLES FROM IMU                 ---
   -------------------------------------------------------------
*/

void update_attitude(axis_t& attitude) {
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
        Serial.println(F("FIFO overflow!"));

        /* Otherwise, check for DMP data ready interrupt (this happens often) */
    } else if (mpu_int_status & 0x02) {
        //100 Hz timing
        //digitalWrite(DEBUG_PIN, HIGH);
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


        /* internal Yaw/Pitch/Roll container and gravity vector */
        static float yaw_pitch_roll[3] = { 0.0 };

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
        // TODO try using [0..2 * M_PI] everywhere without scaling or use quaternions?
        attitude[ROLL_AXIS]  = (int16_t) (yaw_pitch_roll[2] * (500.0 / M_PI));
        attitude[PITCH_AXIS] = (int16_t) (yaw_pitch_roll[1] * (500.0 / M_PI));
        attitude[YAW_AXIS]   = (int16_t) (yaw_pitch_roll[0] * (500.0 / M_PI));
        //digitalWrite(DEBUG_PIN, LOW);
    }
    //frequency 100Hz
    //digitalWrite(DEBUG_PIN, LOW);
}


/*
   ---------------------------------------------------
   ---             CALIBRATE RATES                 ---
   ---------------------------------------------------
*/

static bool calib_rates_ok(axis_t& angular_rates) {
    static bool rate_calibrated = false;

    const int iterations = 50;
    const int tolerance  = 10;

    offset_axis_t accumulator = { 0 };

    for (uint16_t count = 0; count < iterations; count++) {
        update_angular_rates(angular_rates);
        accumulator[ROLL_AXIS]  += angular_rates[ROLL_AXIS];
        accumulator[PITCH_AXIS] += angular_rates[PITCH_AXIS];
        accumulator[YAW_AXIS]   += angular_rates[YAW_AXIS];

        delay(5);
    }

    accumulator[ROLL_AXIS]  /= iterations;
    accumulator[PITCH_AXIS] /= iterations;
    accumulator[YAW_AXIS]   /= iterations;

    Serial.print("Average rate over ");
    Serial.print(iterations);
    Serial.println(" iterations: ");
    Serial.print((uint32_t) accumulator[ROLL_AXIS]);
    Serial.print("\t");
    Serial.print((uint32_t) accumulator[PITCH_AXIS]);
    Serial.print("\t");
    Serial.println((uint32_t) accumulator[YAW_AXIS]);

    rate_calibrated =
        (abs(accumulator[ROLL_AXIS])  < tolerance) &&
        (abs(accumulator[PITCH_AXIS]) < tolerance) &&
        (abs(accumulator[YAW_AXIS])   < tolerance);

    return rate_calibrated;
}

static void calib_rates() {
    Serial.println(F("Calibrating gyro rates, hold still!"));

    uint16_t iterations = 300;

    axis_t angular_rates = { 0, 0, 0 };
    update_angular_rates(angular_rates);

    /* Attempt calibration and check if it (probably) succeeded */
    while (!calib_rates_ok(angular_rates)) {
        gyro_offsets[ROLL_AXIS]  = 0;
        gyro_offsets[PITCH_AXIS] = 0;
        gyro_offsets[YAW_AXIS]   = 0;

        for (uint16_t count = 0; count < iterations; count++) {
            update_raw_rates(angular_rates);
            gyro_offsets[ROLL_AXIS]  += angular_rates[ROLL_AXIS];
            gyro_offsets[PITCH_AXIS] += angular_rates[PITCH_AXIS];
            gyro_offsets[YAW_AXIS]   += angular_rates[YAW_AXIS];

            delay(5);
        }

        gyro_offsets[ROLL_AXIS]  /= iterations;
        gyro_offsets[PITCH_AXIS] /= iterations;
        gyro_offsets[YAW_AXIS]   /= iterations;

        iterations = iterations < 2000 ? iterations + 200 : iterations;
    }
}


/*
   ----------------------------------------------------------------
   ---             MPU INTERRUPT DETECTION ROUTINE              ---
   ----------------------------------------------------------------
*/

/*static*/ void dmp_data_ready() {
    mpu_interrupt = true;
}


/*
   ---------------------------------------------------
   ---             IMU INITIALISATION              ---
   ---------------------------------------------------
*/

void init_mpu6050() {
/* Join I2C bus (I2Cdev library doesn't do this automatically) */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    /* 400kHz I2C clock (200kHz if CPU is 8MHz) */
    TWBR = 12;
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
    dev_status = mpu.dmpInitialize();

    /* Supply your own gyro offsets here, scaled for min sensitivity */
    mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);

    /* TODO Find suitable value */
    mpu.setZAccelOffset(1788);

    /* 100 Hz interrupt rate at 4, 166 Hz at rate 2 */
    mpu.setRate(2);
    /* Make sure initialisation worked (returns 0 if so) */
    if (dev_status == 0) {
        /* Turn on the DMP, now that it's ready */
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        /* Enable Arduino interrupt detection */
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        pinMode(MPU_INTERRUPT_PIN, INPUT);
        attachInterrupt(MPU_INTERRUPT_PIN, dmp_data_ready, RISING);
        mpu_int_status = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        /* Get expected DMP packet size for later comparison */
        packet_size = mpu.dmpGetFIFOPacketSize();
    } else {
        /* Error while init */
        switch (dev_status) {
            case 1: error_blink(DMP_INIT_MEM_LOAD_FAILED, "DMP init error code 1: Initial Memory Load failed!"); break;
            case 2:
                error_blink(DMP_CONF_UPDATES_FAILED, "DMP init error code 2: DMP configuration updates failed!");
                break;
            default: {
                char msg[50] = "DMP init error code     ";
                itoa(dev_status, msg + 20, 10);
                error_blink(DMP_ERROR_UNKNOWN, msg);
                break;
            }
        }
    }
    calib_rates();
}
