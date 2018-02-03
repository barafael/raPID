#include "Arduino.h"

#include <stdint.h>

#include "I2Cdev.h"
#include "Servo.h"

#include "../include/ESCOutput.h"
#include "../include/PIDController.h"
#include "../include/PWMReceiver.h"
#include "../include/arming_state.h"
#include "../include/error_blink.h"
#include "../include/imu.h"
#include "../include/pins.h"
#include "../include/settings.h"
#include "../include/Watchdog.h"

#define TIMING_ANALYSIS
#ifdef TIMING_ANALYSIS
#define time(f)                          \
    {                                    \
        digitalWrite(DEBUG_PIN, HIGH);   \
        f;                               \
        digitalWrite(DEBUG_PIN, LOW);    \
    }
#else
#define time(f) f
#endif

#define notime(f) f


/*
   ----------------------------------------------
   ---             HARDWARE SETUP             ---
   ----------------------------------------------

   MPU6050 Breakout ----- Teensy 3.2
   3.3V ----------------- 3.3V
   GND ------------------ GND
   SDA ------------------ A4/pin 18
   SCL ------------------ A5/pin 19
   INT ------------------ Digital Pin 12 (see pins.h)

   See ../include/pins.h for more pin definitions.
   */

//state_t state = /*DIS*/ARMED;
state_t state = DISARMED;

/* Scaled yaw_pitch_roll to [0, 1000] */
axis_t attitude = { 0, 0, 0 };

/* Angular Rate */
axis_t angular_rate = { 0, 0, 0 };


float pid_output_roll_stbl = 0.0;
float pid_output_roll_rate = 0.0;

float pid_output_pitch_stbl = 0.0;
float pid_output_pitch_rate = 0.0;

float pid_output_yaw_rate = 0.0;

channels_t channels = { 0 };

static void print_attitude(axis_t attitude) {
    for (size_t index = 0; index < 3; index++) {
        Serial.print(attitude[index]);
        Serial.print(F("\t"));
    }
    Serial.println();
}

static void print_channels(channels_t channels) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        Serial.print(channels[index]);
        Serial.print(F("\t"));
    }
    Serial.println();
}

extern "C" int main(void) {
    Serial.begin(9600);

    pinMode(LED_PIN, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);

    delay(1000);

    PWMReceiver receiver(THROTTLE_INPUT_PIN, ROLL_INPUT_PIN,
            PITCH_INPUT_PIN,    YAW_INPUT_PIN,
            AUX1_INPUT_PIN,     AUX2_INPUT_PIN);

    while (!receiver.has_signal()) {
        delay(500);
        Serial.println(F("No receiver signal! Waiting."));
    }

    Serial.println(F("Receiver signal detected, continuing."));

    init_mpu6050();

    PIDParams roll_param_stbl ( 0.1 , 0.0 , 0.0 , 12.0 , 200.0);
    PIDParams roll_param_rate ( 0.1 , 0.0 , 0.0 , 12.0 , 200.0);

    PIDParams pitch_param_stbl( 0.1 , 0.0 , 0.0 , 12.0 , 200.0);
    PIDParams pitch_param_rate( 0.1 , 0.0 , 0.0 , 12.0 , 200.0);

    PIDParams yaw_param_rate  ( 1.0 , 0.0 , 0.0 , 12.0 , 200.0);

    PIDController roll_controller_stbl(&roll_param_stbl);
    PIDController roll_controller_rate(&roll_param_rate);

    PIDController pitch_controller_stbl(&pitch_param_stbl);
    PIDController pitch_controller_rate(&pitch_param_rate);

    PIDController yaw_controller_rate(&yaw_param_rate);

    ESCOutput back_left_out_mixer  (LEFT_SERVO_PIN,  1.0, -0.4, 0.4, 0.0);
    ESCOutput back_right_out_mixer (RIGHT_SERVO_PIN, 1.0, 0.4, 0.4, 0.0);
    ESCOutput front_left_out_mixer (FRONT_SERVO_PIN, 1.0, -0.4, -0.4, 0.0);
    ESCOutput front_right_out_mixer(BACK_SERVO_PIN,  1.0, 0.4, -0.4, 0.0);

    back_left_out_mixer  .shut_off();
    back_right_out_mixer .shut_off();
    front_left_out_mixer .shut_off();
    front_right_out_mixer.shut_off();

    float new_stbl_p = 0.0;
    float new_rate_p = 0.0;

    //roll_controller_stbl.set_enabled(false);
    //roll_controller_rate.set_enabled(false);

    Watchdog dog;

    /* Flight loop */
    while (true) {
        receiver.update(channels);

        //print_channels(channels);

        notime(update_attitude(attitude));

        //print_attitude(attitude);

        notime(update_angular_rates(angular_rate));

        switch (state) {
            case DISARMING:
                Serial.println(F("Disarming!"));
                if (!disarming_input(&channels)) {
                    Serial.println(F("Disarming interrupted! Arming again."));
                    state = ARMED;
                    break;
                } else {
                    state = disarming_complete() ? DISARMED : DISARMING;
                    if (state == DISARMED) {
                        Serial.println(F("Release the hold!"));
                        while (disarming_input(&channels)) {
                            back_left_out_mixer  .shut_off();
                            back_right_out_mixer .shut_off();
                            front_left_out_mixer .shut_off();
                            front_right_out_mixer.shut_off();

                            receiver.update(channels);
                            update_attitude(attitude);
                            update_angular_rates(angular_rate);
                            dog.feed();
                            delay(10);
                        }
                        Serial.println(F("DISARMING COMPLETE!"));
                        break;
                    }
                }

                Serial.println(F("Proceeding to 'ARMED' state actions from 'DISARMING'"));
                /* Keep disarming, but stay armed (no break) */

            case ARMED:
                new_stbl_p = (channels[AUX1_CHANNEL] + 500) / 1000.0 / 8.0;
                new_rate_p = (channels[AUX2_CHANNEL] + 500) / 1000.0 / 8.0;

                //Serial.print(new_stbl_p);
                //Serial.print(F("\t"));
                //Serial.println(new_rate_p);

                //roll_controller_stbl.set_p(new_stbl_p);
                //roll_controller_rate.set_p(new_rate_p);

                //Serial.println(attitude[ROLL_AXIS] - channels[ROLL_CHANNEL]);
                pid_output_roll_stbl = roll_controller_stbl.  compute(attitude[ROLL_AXIS], channels[ROLL_CHANNEL]);
                //Serial.println(pid_output_roll_stbl);

                pid_output_roll_rate = roll_controller_rate.  compute(angular_rate[ROLL_AXIS], -15 * pid_output_roll_stbl);
                //Serial.println(pid_output_roll_rate);

                pid_output_pitch_stbl = pitch_controller_stbl.compute(attitude[PITCH_AXIS], channels[PITCH_CHANNEL]);

                pid_output_pitch_rate = pitch_controller_rate.compute(angular_rate[PITCH_AXIS], -15 * pid_output_pitch_stbl);

                /* Yaw needs rate only - yaw stick controls rate of rotation, there is no fixed reference */
                pid_output_yaw_rate = yaw_controller_rate.    compute(angular_rate[YAW_AXIS], channels[YAW_CHANNEL]);

                back_left_out_mixer  .apply(channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                back_right_out_mixer .apply(channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                front_left_out_mixer .apply(channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                front_right_out_mixer.apply(channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);

//#define DEBUG_COL
#ifdef DEBUG_COL
                Serial.print(F("setp:"));
                Serial.print(channels[ROLL_CHANNEL]);
                Serial.print(F("\troll-angl:"));
                Serial.print(attitude[ROLL_AXIS]);
                Serial.print(F("\tpid_output_roll_stbl:"));
                Serial.print(pid_output_roll_stbl);
                Serial.print(F("\tpid_output_roll_rate:"));
                Serial.println(pid_output_roll_rate);
#endif

                /* State can be DISARMING because in that state everything from ARMED state must happen anyway */
                if (state != DISARMING && disarming_input(&channels)) {
                    Serial.println(F("Initializing Disarm!"));
                    state = DISARMING;
                    disarm_init();
                }
                break;

            case ARMING:
                Serial.println(F("Arming!"));
                if (!arming_input(&channels)) {
                    Serial.println(F("Arming interrupted! Disarming again."));
                    state = DISARMED;
                    break;
                } else {
                    state = arming_complete() ? ARMED : ARMING;
                    if (state == ARMED) {
                        Serial.println(F("Release the hold!"));
                        while (arming_input(&channels)) {
                            /* Still don't fire the motors up */
                            back_left_out_mixer  .shut_off();
                            back_right_out_mixer .shut_off();
                            front_left_out_mixer .shut_off();
                            front_right_out_mixer.shut_off();

                            receiver.update(channels);
                            update_attitude(attitude);
                            update_angular_rates(angular_rate);
                            dog.feed();
                            delay(10);
                        }
                        Serial.println(F("ARMING COMPLETE!"));
                        break;
                    }
                }
                Serial.println(F("Proceeding to 'DISARMED' state actions from 'ARMING'"));
                /* Keep arming, but stay disarmed (no break) */

            case DISARMED:
                back_left_out_mixer  .shut_off();
                back_right_out_mixer .shut_off();
                front_left_out_mixer .shut_off();
                front_right_out_mixer.shut_off();

                if (state != ARMING && arming_input(&channels)) {
                    state = ARMING;
                    arm_init();
                }
                break;

            case CONFIG:
                Serial.println(F("CONFIG!"));
                state = DISARMED;
                break;
            default:
                Serial.println(F("Unimplemented state! Will disarm."));
                state = DISARMED;
                break;
        }

        static bool blink_state = false;

        /* Blink LED to indicate activity */
        blink_state = !blink_state;
        digitalWrite(LED_PIN, blink_state);
        dog.feed();
    }
}
