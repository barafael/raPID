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
#include "../include/watchdog.h"
#include "../include/rc_control.h"


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

Servo left_ppm;
Servo right_ppm;
uint16_t left_throttle;
uint16_t right_throttle;
uint16_t throttle;
uint16_t receiver_in[NUM_CHANNELS] = { 0 };

extern "C" int main(void) {
    pinMode(LED_PIN, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);

    serial_begin(9600);

    init_rx_interrupts();

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
