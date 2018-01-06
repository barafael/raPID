#include "Arduino.h"
#include "WProgram.h"

#include <stdint.h>

#include "I2Cdev.h"
#include "Servo.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#include "../interface/error_handling.h"
#include "../interface/imu.h"
#include "../interface/pid.h"
#include "../interface/pid_controller.h"
#include "../interface/pins.h"
#include "../interface/rc_control.h"
#include "../interface/output_mixer.h"
#include "../interface/receiver.h"
#include "../interface/settings.h"
#include "../interface/state.h"
#include "../interface/state_change.h"
#include "../interface/watchdog.h"

#define TIMING_ANALYSIS
#ifdef TIMING_ANALYSIS
#define time(f)                                                                                                        \
    {                                                                                                                  \
        digitalWrite(DEBUG_PIN, HIGH);                                                                                 \
        f;                                                                                                             \
        digitalWrite(DEBUG_PIN, LOW);                                                                                  \
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

channels_t receiver_in;

/* TODO: enumify? */
size_t flight_mode_index = 0;

extern "C" int main(void) {
    Serial.begin(9600);

    pinMode(LED_PIN, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);

    init_rx_interrupts();

    init_mpu6050();

    init_watchdog();

    pid_controller roll_controller_rate = pid_controller(1.0, 0.0, 0.0, 12.0, 200.0);
    pid_controller roll_controller_stbl = pid_controller(1.0, 0.0, 0.0, 12.0, 200.0);

    pid_controller pitch_controller_rate = pid_controller(1.0, 0.0, 0.0, 12.0, 200.0);
    pid_controller pitch_controller_stbl = pid_controller(1.0, 0.0, 0.0, 12.0, 200.0);

    mixer_t roll_left_mixer;
    roll_left_mixer.throttle_vol = 100;
    roll_left_mixer.volumes = { 100, 0, 0 };
    Output_mixer out_mixer_left(SERVO, LEFT_SERVO_PIN, roll_left_mixer);

    mixer_t roll_right_mixer;
    roll_right_mixer.throttle_vol = 100;
    roll_right_mixer.volumes = { -100, 0, 0 };
    Output_mixer out_mixer_right(SERVO, RIGHT_SERVO_PIN, roll_right_mixer);

    mixer_t pitch_front_mixer;
    pitch_front_mixer.throttle_vol = 100;
    pitch_front_mixer.volumes = { 0, 100, 0 };
    Output_mixer out_mixer_front(SERVO, FRONT_SERVO_PIN, pitch_front_mixer);

    mixer_t pitch_back_mixer;
    pitch_back_mixer.throttle_vol = 100;
    pitch_back_mixer.volumes = { 0, -100, 0 };
    Output_mixer out_mixer_back(SERVO, BACK_SERVO_PIN, pitch_back_mixer);

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
                            out_mixer_left.shut_off();
                            out_mixer_right.shut_off();
                            out_mixer_front.shut_off();
                            out_mixer_back.shut_off();

                            read_receiver(&receiver_in);
                            read_abs_angles(&attitude);
                            read_angular_rates(&angular_rate);
                            feed_the_dog();
                            delay(10);
                        }
                        Serial.println("DISARMING COMPLETE!");
                        break;
                    }
                }

                Serial.println("Proceeding to 'ARMED' state actions from 'DISARMING'");
                /* Keep disarming, but stay armed */

            case ARMED:
                pid_output_roll_stbl = roll_controller_stbl.compute(micros(), attitude.roll, receiver_in.channels[ROLL_CHANNEL] - 1000).sum;

                // setpoint_rate = receiver_in[ROLL_CHANNEL] - 1500.0;

                pid_output_roll_rate = roll_controller_rate.compute(micros(), angular_rate.roll, -15 * pid_output_roll_stbl).sum;

                pid_output_pitch_stbl = pitch_controller_stbl.compute(micros(), attitude.pitch, receiver_in.channels[PITCH_CHANNEL] - 1000).sum;

                pid_output_pitch_rate = pitch_controller_rate.compute(micros(), angular_rate.pitch, -15 * pid_output_pitch_stbl).sum;

                out_mixer_left. apply(receiver_in.channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, 0.0);
                out_mixer_right.apply(receiver_in.channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, 0.0);
                out_mixer_front.apply(receiver_in.channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, 0.0);
                out_mixer_back. apply(receiver_in.channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, 0.0);

#define DEBUG_COL
#ifdef DEBUG_COL
                Serial.print("thr:");
                Serial.print(receiver_in.channels[THROTTLE_CHANNEL]);
                Serial.print("\tsetp:");
                Serial.print(pid_output_roll_stbl);
                Serial.print("\tr-angl:");
                Serial.print(attitude.roll);
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
                            /* Still don't fire the motors up */
                            out_mixer_left.shut_off();
                            out_mixer_right.shut_off();
                            out_mixer_front.shut_off();
                            out_mixer_back.shut_off();

                            read_receiver(&receiver_in);
                            read_abs_angles(&attitude);
                            read_angular_rates(&angular_rate);
                            feed_the_dog();
                            delay(10);
                        }
                        Serial.println("ARMING COMPLETE!");
                        break;
                    }
                }
                Serial.println("Proceeding to 'DISARMED' state actions from 'ARMING'");
                /* Keep arming, but stay disarmed */

            case DISARMED:
                out_mixer_left.shut_off();
                out_mixer_right.shut_off();
                out_mixer_front.shut_off();
                out_mixer_back.shut_off();

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
