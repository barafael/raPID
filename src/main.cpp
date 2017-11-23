#include <Arduino.h>
#include "WProgram.h"

#include <stdint.h>

#include "Servo.h"
#include "I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#include "../include/error_handling.h"
#include "../include/settings.h"
#include "../include/pins.h"
#include "../include/serial_debug.h"
#include "../include/read_receiver.h"
#include "../include/imu.h"
#include "../include/pid.h"


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


static bool blink_state = false;
static double setpoint_rate;

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


double pid_output_roll = 0.0;
double pid_output_roll_rate = 0.0;

/*
   ————————————————————————————————————————————————————
   ———           SERVO GLOBAL VARIABLES             ———
   ————————————————————————————————————————————————————
*/

Servo left_ppm;
Servo right_ppm;
uint16_t left_throttle;
uint16_t right_throttle;
uint16_t throttle;

/* Arm ESC's with a long low pulse */

void arm_ESC() {
    serial_println("Initialising ESCs: 1000ms pulse");
    left_ppm.writeMicroseconds(1000);
    right_ppm.writeMicroseconds(1000);
    delay(1000);
    serial_println("Initialised ESCs");
}


uint16_t receiver_in[NUM_CHANNELS] = { 0 };


void inline watchdog_init() {
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);
    /* Enable WDG */
    WDOG_STCTRLH = 0x0001;
    /* The next 2 lines sets the time-out value.
     * This is the value that the watchdog timer compare itself to.
     * */
    WDOG_TOVALL = 200;
    WDOG_TOVALH = 0;
    /* This sets prescale clock so that the watchdog timer ticks at
     * 1kHZ instead of the default 1kHZ/4 = 200 HZ
     * */
    WDOG_PRESC = 4;
}

void inline feed_the_dog() {
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}


extern "C" int main(void) {
    pinMode(LED_PIN, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);

    serial_begin(9600);

    init_rc_interrupts();

    left_ppm.attach(LEFT_SERVO_PIN);
    right_ppm.attach(RIGHT_SERVO_PIN);

    arm_ESC();

    init_MPU6050();

    watchdog_init();

    while (1) {
        read_receiver();

        // digitalWrite(DEBUG_PIN, HIGH);
        read_angular_rates();
        // digitalWrite(DEBUG_PIN, LOW);

        setpoint_rate = receiver_in[ROLL_CHANNEL] - 1500.0;
        calculate_PID_rate(setpoint_rate, gyro_axis[ROLL_RATE]);

        // digitalWrite(DEBUG_PIN, HIGH);
        read_abs_angles();
        // digitalWrite(DEBUG_PIN, LOW);

        // calculate_PID_stabilize(receiver_in[ROLL_CHANNEL],
        //        attitude[ROLL_ANGLE], gyro_axis[ROLL_RATE]);

        // print_receiver();

        /*
        int value = (gyro_axis[ROLL_RATE] + 2000) * (255.0/4000.0);
        analogWrite(DEBUG_PIN, value);
        */

        throttle = receiver_in[THROTTLE_CHANNEL];

        left_throttle  = throttle + pid_output_roll_rate;
        right_throttle = throttle - pid_output_roll_rate;

        left_throttle = left_throttle < 1000 ? 1000 : left_throttle;
        right_throttle = right_throttle < 1000 ? 1000 : right_throttle;

        left_throttle = left_throttle > 2000 ? 2000 : left_throttle;
        right_throttle = right_throttle > 2000 ? 2000 : right_throttle;

        left_ppm.writeMicroseconds(left_throttle);
        right_ppm.writeMicroseconds(right_throttle);

        #define DEBUG_COL
#ifdef DEBUG_COL
        serial_print("thr:");
        serial_print(throttle);
        serial_print("\tsetp:");
        serial_print(setpoint_rate);
        serial_print("\tr-angl:");
        serial_print(attitude[ROLL_ANGLE]);
        serial_print("\tleft:");
        serial_print(left_throttle);
        serial_print("\tright:");
        serial_print(right_throttle);
        serial_print("\tr-p-out:");
        serial_println(pid_output_roll_rate);
#endif

        /* Blink LED to indicate activity */
        blink_state = !blink_state;
        digitalWrite(LED_PIN, blink_state);
        feed_the_dog();
    }
}

