#include "../teensy3/Arduino.h"
#include "../teensy3/WProgram.h"

#include <stdint.h>

#include "Servo.h"
#include "I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#include "../interface/error_handling.h"
#include "../interface/settings.h"
#include "../interface/pins.h"
#include "../interface/receiver.h"
#include "../interface/imu.h"
#include "../interface/pid.h"
#include "../interface/watchdog.h"
#include "../interface/rc_control.h"
#include "../interface/state.h"
#include "../interface/state_change.h"
#include "../interface/pid_controller.h"

#define TIMING_ANALYSIS
#ifdef TIMING_ANALYSIS
#define time(f) {\
    digitalWrite(DEBUG_PIN, HIGH); \
    f;\
    digitalWrite(DEBUG_PIN, LOW);\
}
#else
#define time(f) f
#endif

#define notime(f) f


/*
   ——————————————————————————————————————————————
   ———             HARDWARE SETUP             ———
   ——————————————————————————————————————————————

   MPU6050 Breakout ----- Teensy 3.2
   3.3V ----------------- 3.3V
   GND ------------------ GND
   SDA ------------------ A4/pin 18
   SCL ------------------ A5/pin 19
   INT ------------------ Digital Pin 12 (see pins.h)

   PPM from RC RX go to pins 8, 9, 10, 11 (see pins.h)
   Output PPM to ESC's: pins 21, 22
   */

state_t state = DISARMED;

/* Scaled yaw_pitch_roll to [0, 1000]
*/

axis_t attitude = { 0, 0, 0 };

/* Angular Rate
*/
axis_t angular_rate = { 0, 0, 0 };


float pid_output_roll_stbl = 0.0;
float pid_output_roll_rate = 0.0;

float pid_output_pitch_stbl = 0.0;
float pid_output_pitch_rate = 0.0;

Servo left_ppm;
Servo right_ppm;
Servo front_ppm;
Servo back_ppm;

uint16_t left_throttle;
uint16_t right_throttle;
uint16_t front_throttle;
uint16_t back_throttle;

channels_t receiver_in;

/* TODO: enumify? */
size_t flight_mode_index = 0;

extern "C" int main(void) {
    Serial.begin(9600);

    pinMode(LED_PIN, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);

    left_ppm.attach(LEFT_SERVO_PIN);
    right_ppm.attach(RIGHT_SERVO_PIN);

    front_ppm.attach(FRONT_SERVO_PIN);
    back_ppm.attach(BACK_SERVO_PIN);

    notime(arm_ESC(&left_ppm, &right_ppm, &front_ppm, &back_ppm));

    init_rx_interrupts();

    init_mpu6050();

    init_watchdog();

    pid_controller roll_controller_rate = pid_controller(1.0, 0.0, 1.0, 12.0, 200.0);
    pid_controller roll_controller_stbl = pid_controller(1.0, 0.0, 1.0, 12.0, 200.0);

    pid_controller pitch_controller_rate = pid_controller(1.0, 0.0, 0.0, 12.0, 200.0);
    pid_controller pitch_controller_stbl = pid_controller(1.0, 0.0, 0.0, 12.0, 200.0);

    while (1) {
        notime(read_receiver(&receiver_in));

        notime(read_abs_angles(&attitude));

        notime(read_angular_rates(&angular_rate));

        switch (state) {
            case DISARMING:
                Serial.println("Disarming!");
                if (!disarming_input(&receiver_in)) {
                    Serial.println("Disarming interrupted! Arming again.");
                    state = ARMED;
                    break;
                } else {
                    state = disarming_complete() ? DISARMED : DISARMING;
                    if (state == DISARMED) {
                        Serial.println("Release the hold!");
                        while (disarming_input(&receiver_in)) {
                            left_ppm.writeMicroseconds(1000);
                            right_ppm.writeMicroseconds(1000);

                            front_ppm.writeMicroseconds(1000);
                            back_ppm.writeMicroseconds(1000);

                            read_receiver(&receiver_in);
                            read_abs_angles(&attitude);
                            read_angular_rates(&angular_rate);
                            feed_the_dog();
                            delayMicroseconds(5000);
                        }
                        Serial.println("DISARMING COMPLETE!");
                        break;
                    }
                }

                Serial.println("Proceeding to 'ARMED' state actions from 'DISARMING'");
                /* Keep disarming, but also stay armed */

            case ARMED:

                pid_output_roll_stbl = roll_controller_stbl.compute(micros(), attitude.roll, receiver_in.channels[ROLL_CHANNEL] - 1000).sum;

                // setpoint_rate = receiver_in[ROLL_CHANNEL] - 1500.0;

                pid_output_roll_rate = roll_controller_rate.compute(micros(), angular_rate.roll, -15 * pid_output_roll_stbl).sum;

                pid_output_pitch_stbl = pitch_controller_stbl.compute(micros(), attitude.pitch, receiver_in.channels[PITCH_CHANNEL] - 1000).sum;

                pid_output_pitch_rate = pitch_controller_rate.compute(micros(), angular_rate.pitch, -15 * pid_output_pitch_stbl).sum;

                left_throttle  = receiver_in.channels[THROTTLE_CHANNEL] + pid_output_roll_rate;
                right_throttle = receiver_in.channels[THROTTLE_CHANNEL] - pid_output_roll_rate;

                front_throttle = receiver_in.channels[THROTTLE_CHANNEL] + pid_output_pitch_rate;
                back_throttle  = receiver_in.channels[THROTTLE_CHANNEL] - pid_output_pitch_rate;

                left_throttle = left_throttle < 1000 ? 1000 : left_throttle;
                left_throttle = left_throttle > 2000 ? 2000 : left_throttle;

                right_throttle = right_throttle < 1000 ? 1000 : right_throttle;
                right_throttle = right_throttle > 2000 ? 2000 : right_throttle;

                front_throttle = front_throttle < 1000 ? 1000 : front_throttle;
                front_throttle = front_throttle > 2000 ? 2000 : front_throttle;

                back_throttle = back_throttle > 2000 ? 2000 : back_throttle;
                back_throttle = back_throttle < 1000 ? 1000 : back_throttle;

                left_ppm.writeMicroseconds(left_throttle);
                right_ppm.writeMicroseconds(right_throttle);

                front_ppm.writeMicroseconds(front_throttle);
                back_ppm.writeMicroseconds(back_throttle);

#define DEBUG_COL
#ifdef DEBUG_COL
                Serial.print("thr:");
                Serial.print(receiver_in.channels[THROTTLE_CHANNEL]);
                Serial.print("\tsetp:");
                Serial.print(pid_output_roll_stbl);
                Serial.print("\tr-angl:");
                Serial.print(attitude.roll);
                Serial.print("\tleft:");
                Serial.print(left_throttle);
                Serial.print("\tright:");
                Serial.print(right_throttle);
                Serial.print("\tr-p-out:");
                Serial.print(pid_output_roll_stbl);
                Serial.print("\tr-p_rate-out:");
                Serial.println(pid_output_roll_rate);
#endif

                if (state != DISARMING && disarming_input(&receiver_in)) {
                    Serial.println("Initializing Disarm!");
                    state = DISARMING;
                    disarm_init();
                }
                break;

            case ARMING:
                Serial.println("Arming!");
                if (!arming_input(&receiver_in)) {
                    Serial.println("Arming interrupted! Disarming again.");
                    state = DISARMED;
                    break;
                } else {
                    state = arming_complete() ? ARMED : ARMING;
                    if (state == ARMED) {
                        Serial.println("Release the hold!");
                        while (arming_input(&receiver_in)) {
                            read_receiver(&receiver_in);
                            read_abs_angles(&attitude);
                            read_angular_rates(&angular_rate);
                            feed_the_dog();
                            delayMicroseconds(5000);
                        }
                        Serial.println("ARMING COMPLETE!");
                        break;
                    }
                }
                Serial.println("Proceeding to 'DISARMED' state actions from 'ARMING'");

                /* Keep arming, but also stay disarmed */

            case DISARMED:

                left_ppm.writeMicroseconds(1000);
                right_ppm.writeMicroseconds(1000);

                front_ppm.writeMicroseconds(1000);
                back_ppm.writeMicroseconds(1000);

                if (state != ARMING && arming_input(&receiver_in)) {
                    state = ARMING;
                    arm_init();
                }
                break;

            case CONFIG:
                Serial.println("CONFIG!");
                state = DISARMED;
                break;
            default:
                Serial.println("Unimplemented state! Will disarm.");
                state = DISARMED;
                break;
        }

        static bool blink_state = false;

        /* Blink LED to indicate activity */
        blink_state = !blink_state;
        digitalWrite(LED_PIN, blink_state);
        feed_the_dog();
    }
}
