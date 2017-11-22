#include <Arduino.h>
#include "../teensy3/WProgram.h"

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

   PPM from RC RX go to pins 9, 10, 11, 12 (see pins.h)
   Output PPM to ESC's: pins 20, 21
*/


#define ENABLE_DEBUG_PRINT

#ifdef ENABLE_DEBUG_PRINT
    #define serial_println(a); (Serial.println(a)); Serial.flush();
    #define serial_print(a); (Serial.print(a)); Serial.flush();
    #define serial_begin(a); (Serial.begin(a)); Serial.flush();
#else
    #define serial_println(a);
    #define serial_print(a);
    #define serial_begin(a);
#endif

#include <../../tools/arm/arm-none-eabi/include/stdint.h>

#include "error_handling.h"
#include "settings.h"
#include "pins.h"
#include "serial_debug.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_9Axis_MotionApps41.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "Servo.h"


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
MPU6050 mpu;

static const uint8_t mpu_address = 0x68;

static const uint8_t LED_PIN = 13;
bool blink_state = false;

/* Holds actual interrupt status byte from MPU */
uint8_t mpu_int_status;

/* Return status after each device operation
   (0 = success, !0 = error) */
uint8_t dev_status;

/* Expected DMP packet size (default is 42 bytes) */
uint16_t packet_size;

uint16_t fifo_count;
uint8_t fifo_buffer[64];


/*
   —————————————————————————————————————————————————
   ———          ORIENTATION/MOTION VARS          ———
   —————————————————————————————————————————————————
*/

Quaternion q;        // [w, x, y, z]    quaternion container
VectorInt16 aa;      // [x, y, z]       accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]       gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]       world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]       gravity vector

/* Euler angle container
 * [psi, theta, phi]
 */
float euler[3] = { 0.0 };

/* Yaw/Pitch/Roll container and gravity vector
 * [yaw, pitch, roll]
 */
float yaw_pitch_roll[3] = { 0.0 };

/* Scaled yaw_pitch_roll to [0, 1000]
 * [yaw, pitch, roll]
 */
int16_t attitude[3] = { 0 };

/* Angular Rates
 * [yaw_rate, pitch_rate, roll_rate]
 */
int16_t gyro_axis[3] = { 0 };

/* Angular Rate calibration offsets
 * [yaw_offset, pitch_offset, roll_offset]
 */
int64_t gyro_axis_cal[3] = { 0 };


/*
   ————————————————————————————————————————————————————————————————
   ———             MPU INTERRUPT DETECTION ROUTINE              ———
   ————————————————————————————————————————————————————————————————
*/

/* Indicates whether MPU interrupt pin has gone high */
volatile bool mpu_interrupt = false;

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
    TWBR = 24;
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
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    /* Make sure initialisation worked (returns 0 if so) */
    if (dev_status == 0) {
        /* Turn on the DMP, now that it's ready */
        serial_println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        /* Enable Arduino interrupt detection */
        serial_println(
            F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        pinMode(2, INPUT);
        attachInterrupt(2, dmp_data_ready, RISING);
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
}


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

        /* Supply your own gyro offsets here, scaled for min sensitivity */
        /*
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
        */
        iterations = iterations < 2000 ? iterations + 200 : iterations;
    }
}


/*
   —————————————————————————————————————————————————————————————
   ———             FETCH ABS ANGLES FROM IMU                 ———
   —————————————————————————————————————————————————————————————
*/

void read_MPU_data() {
    /* Reset interrupt flag and get INT_STATUS byte */
    mpu_interrupt = false;
    mpu_int_status = mpu.getIntStatus();

    /* Get current FIFO count */
    fifo_count = mpu.getFIFOCount();

    /* Check for overflow (this should be rare) */
    if ((mpu_int_status & 0x10) || fifo_count == 1024) {
        /* reset so we can continue cleanly */
        mpu.resetFIFO();
        serial_println(F("FIFO overflow!"));

        /* Otherwise, check for DMP data ready interrupt (this happens often) */
    } else if (mpu_int_status & 0x02) {
        /* Wait for correct available data length, should be a VERY short wait */
        while (fifo_count < packet_size) {
            fifo_count = mpu.getFIFOCount();
        }

        /* Read a packet from FIFO */
        mpu.getFIFOBytes(fifo_buffer, packet_size);

        /* Track FIFO count here in case there is > 1 packet available */
        /* (this lets us immediately read more without waiting for an interrupt) */
        fifo_count -= packet_size;

        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(yaw_pitch_roll, &q, &gravity);

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

        for (size_t index = YAW_ANGLE; index <= ROLL_ANGLE; index++) {
            attitude[index] = (yaw_pitch_roll[index] + M_PI) * (1000 / (2 * M_PI));
        }
    }
}


/*
   ————————————————————————————————————————————————————————————————
   ———             RECEIVER READ GLOBAL VARIABLES               ———
   ————————————————————————————————————————————————————————————————
*/

static volatile byte input_flags;

/* The servo interrupt writes to this variable and the main loop reads */
volatile uint16_t receiver_in_shared[NUM_CHANNELS];

int16_t receiver_in[NUM_CHANNELS];

/* Written by interrupt when HIGH value is read */
uint32_t receiver_in_start[NUM_CHANNELS];

/* Read each new value, indicated by the corresponding bit set in input_flags */
void read_receiver() {
    noInterrupts();
    for (size_t channel = 0; channel < NUM_CHANNELS; channel++) {
        if (input_flags & (1 << channel)) {
            receiver_in[channel] = receiver_in_shared[channel] - 1000;
        }
    }
    interrupts();
}


/*
   ————————————————————————————————————————————————————
   ———           SERVO GLOBAL VARIABLES             ———
   ————————————————————————————————————————————————————
*/

Servo left_ppm;
Servo right_ppm;
int16_t left_throttle;
int16_t right_throttle;
int16_t throttle;

/* Arm ESC's with a long low pulse */

void arm_ESC() {
    serial_println("Initialising ESCs: 1000ms pulse");
    left_ppm.writeMicroseconds(1000);
    right_ppm.writeMicroseconds(1000);
    delay(1500);
    serial_println("Initialised ESCs");
}

void read_throttle();
void read_roll();
void read_pitch();
void read_yaw();


/*
   ————————————————————————————————————————————————————
   ———        PID VARIABLES AND COEFFICIENTS        ———
   ————————————————————————————————————————————————————
*/

float pid_output_roll;

double pid_p_gain_roll = 0.25;
double pid_i_gain_roll = 0.0015;
double pid_d_gain_roll = 0.03;
int pid_max_roll = 400;
int pid_roll_integral_limit = 10;

double pid_error;
double pid_last_error;

double p_term;
double i_term;
double d_term;

float pid_roll_setpoint;

/* Calculate PID output based on absolute angle in attitude[] */
void calculate_PID_absolute() {
    pid_roll_setpoint = receiver_in[ROLL_CHANNEL];
    pid_error = attitude[ROLL_ANGLE] - pid_roll_setpoint;

    p_term = pid_p_gain_roll * pid_error;

    i_term += (pid_i_gain_roll * pid_error);
    if (i_term > pid_roll_integral_limit) i_term = pid_roll_integral_limit;
    else if (i_term < (pid_roll_integral_limit * -1)) i_term = (pid_roll_integral_limit * -1);

    d_term = pid_d_gain_roll * gyro_axis[ROLL_RATE];
    //d_term = pid_d_gain_roll * (pid_error - pid_last_error);

    pid_output_roll = p_term + i_term + d_term;

    if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

    pid_last_error = pid_error;
}

/* Calculate PID output based on angular rate */
void calculatePID_angular_rate() {
    pid_roll_setpoint = receiver_in[ROLL_CHANNEL];
    pid_error = gyro_axis[ROLL_RATE] - pid_roll_setpoint;

    p_term = pid_p_gain_roll * pid_error;

    i_term += (pid_i_gain_roll * pid_error);
    if (i_term > pid_roll_integral_limit) i_term = pid_roll_integral_limit;
    else if (i_term < (pid_roll_integral_limit * -1)) i_term = (pid_roll_integral_limit * -1);

    d_term = pid_d_gain_roll * (pid_error - pid_last_error);

    pid_output_roll = p_term + i_term + d_term;

    if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

    pid_last_error = pid_error;
}

void inline watchdog_init() {
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);
    /* Enable WDG */
    WDOG_STCTRLH = 0x0001;
    /* The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to. */
    WDOG_TOVALL = 200;
    WDOG_TOVALH = 0;
    /* This sets prescale clock so that the watchdog timer ticks at 1kHZ instead of the default 1kHZ/4 = 200 HZ */
    // WDOG_PRESC = 0;
}

void inline kick_the_dog() {
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}


extern "C" int main(void) {
    pinMode(LED_PIN, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);

    serial_begin(115200);

    left_ppm.attach(LEFT_SERVO_PIN);
    right_ppm.attach(RIGHT_SERVO_PIN);

    arm_ESC();

    init_MPU6050();

    calib_rates();

    /* The pinMode should be correct by default, set it anyway */
    pinMode(THROTTLE_INPUT_PIN, INPUT);
    pinMode(ROLL_INPUT_PIN,     INPUT);
    pinMode(PITCH_INPUT_PIN,    INPUT);
    pinMode(YAW_INPUT_PIN,      INPUT);

    /* On each CHANGE on an input pin, an interrupt handler is called */
    attachInterrupt(THROTTLE_INPUT_PIN, read_throttle, CHANGE);
    attachInterrupt(ROLL_INPUT_PIN,     read_roll,     CHANGE);
    attachInterrupt(PITCH_INPUT_PIN,    read_pitch,    CHANGE);
    attachInterrupt(YAW_INPUT_PIN,      read_yaw,      CHANGE);

    //watchdog_init();

    while(1) {
        read_angular_rates();

        read_receiver();

        calculate_PID_absolute();

        /* wait for MPU interrupt or extra packet(s) available */
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpu_interrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        if(mpu_interrupt || fifo_count >= packet_size) {
            read_MPU_data();
        }

        /*
        int value = (gyro_axis[ROLL_RATE] + 2000) * (255.0/4000.0);
        analogWrite(DEBUG_PIN, value);
        */

        throttle = receiver_in[THROTTLE_CHANNEL] + 10;

        left_throttle  = throttle + pid_output_roll;
        right_throttle = throttle - pid_output_roll;

        left_throttle = left_throttle < 0 ? 0 : left_throttle;
        right_throttle = right_throttle < 0 ? 0 : right_throttle;

        left_throttle = left_throttle > 1000 ? 1000 : left_throttle;
        right_throttle = right_throttle > 1000 ? 1000 : right_throttle;

        serial_print("thr:");
        serial_print(throttle);
        serial_print("\tsetp:");
        serial_print(receiver_in[ROLL_CHANNEL]);
        serial_print("\tr-angl:");
        serial_print(attitude[ROLL_ANGLE]);
        serial_print("\tleft:");
        serial_print(left_throttle);
        serial_print("\tright:");
        serial_print(right_throttle);
        serial_print("\tr-p-out:");
        serial_println(pid_output_roll);

        left_ppm.writeMicroseconds(left_throttle + 1000);
        right_ppm.writeMicroseconds(right_throttle + 1000);

        /* Blink LED to indicate activity */
        blink_state = !blink_state;
        digitalWrite(DEBUG_PIN, blink_state);

        //kick_the_dog();
    }
}


/*
   ————————————————————————————————————————————————————————————————
   ———             RECEIVER READ INTERRUPT ROUTINES             ———
   ————————————————————————————————————————————————————————————————
*/

void read_throttle() {
    if (digitalRead(THROTTLE_INPUT_PIN) == HIGH) {
        receiver_in_start[THROTTLE_CHANNEL] = micros();
    } else {
        receiver_in_shared[THROTTLE_CHANNEL] =
            (uint16_t)(micros() - receiver_in_start[THROTTLE_CHANNEL]);
        input_flags |= 1 << THROTTLE_CHANNEL;
    }
}

void read_roll() {
    if (digitalRead(ROLL_INPUT_PIN) == HIGH) {
        receiver_in_start[ROLL_CHANNEL] = micros();
    } else {
        receiver_in_shared[ROLL_CHANNEL] =
            (uint16_t)(micros() - receiver_in_start[ROLL_CHANNEL]);
        input_flags |= 1 << ROLL_CHANNEL;
    }
}

void read_pitch() {
    if (digitalRead(PITCH_INPUT_PIN) == HIGH) {
        receiver_in_start[PITCH_CHANNEL] = micros();
    } else {
        receiver_in_shared[PITCH_CHANNEL] =
            (uint16_t)(micros() - receiver_in_start[PITCH_CHANNEL]);
        input_flags |= 1 << PITCH_CHANNEL;
    }
}

void read_yaw() {
    if (digitalRead(YAW_INPUT_PIN) == HIGH) {
        receiver_in_start[YAW_CHANNEL] = micros();
    } else {
        receiver_in_shared[YAW_CHANNEL] =
            (uint16_t)(micros() - receiver_in_start[YAW_CHANNEL]);
        input_flags |= 1 << YAW_CHANNEL;
    }
}


/*
   ————————————————————————————————————————————————————
   ———             SERIAL DEBUG OUTPUT              ———
   ————————————————————————————————————————————————————
*/

void print_yaw_pitch_roll() {
    Serial.print(F("yaw_pitch_roll\t"));
    Serial.print(yaw_pitch_roll[YAW_ANGLE]);
    Serial.print(F("\t"));
    Serial.print(yaw_pitch_roll[PITCH_ANGLE]);
    Serial.print(F("\t"));
    Serial.println(yaw_pitch_roll[ROLL_ANGLE]);
}

void print_attitude() {
    Serial.print(F("Attitude\t"));
    Serial.print(attitude[YAW_ANGLE]);
    Serial.print(F("\t"));
    Serial.print(attitude[PITCH_ANGLE]);
    Serial.print(F("\t"));
    Serial.println(attitude[ROLL_ANGLE]);
}

void print_receiver() {
    Serial.print(receiver_in[THROTTLE_CHANNEL]);
    Serial.print(F("\t"));
    Serial.print(receiver_in[ROLL_CHANNEL]);
    Serial.print(F("\t"));
    Serial.print(receiver_in[PITCH_CHANNEL]);
    Serial.print(F("\t"));
    Serial.println(receiver_in[YAW_CHANNEL]);
}

void print_angular_rates() {
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
    /* truncate v to specified number of places */
    value = value & mask;

    while (num_places) {

        if (value & (0x0001 << (num_places - 1))) {
            serial_print(F("1"));
        } else {
            serial_print(F("0"));
        }

        --num_places;
        if (((num_places % 4) == 0) && (num_places != 0)) {
            serial_print("_");
        }
    }
    serial_println();
}

void print_all(int num, ...) {
    char* line;
    va_list argList;
    va_start(argList, num);

    for(; num; num--) {
        line = va_arg(argList, char*);
        Serial.print("line: ");
        Serial.println(line);
    }
    va_end(argList);
}

