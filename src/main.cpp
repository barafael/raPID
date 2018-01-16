#include "Arduino.h"
#include "WProgram.h"

#include <stdint.h>

#include "I2Cdev.h"
#include "Servo.h"

#include "../interface/error_blink.h"
#include "../interface/imu.h"
#include "../interface/PIDController.h"
#include "../interface/pins.h"
#include "../interface/Output.h"
#include "../interface/PWMReceiver.h"
#include "../interface/settings.h"
#include "../interface/state.h"
#include "../interface/watchdog.h"

#define TIMING_ANALYSIS
#ifdef TIMING_ANALYSIS
#define time(f)                           \
    {                                     \
        digitalWrite(DEBUG_PIN, HIGH);    \
        f;                                \
        digitalWrite(DEBUG_PIN, LOW);     \
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
   Output signal to ESCs/Servos: pins 20, 21, 22, 23 (see pins.h)
   */

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

channels_t receiver_in = { 0 };

PWMReceiver receiver(THROTTLE_INPUT_PIN, ROLL_INPUT_PIN,
                     PITCH_INPUT_PIN,    YAW_INPUT_PIN,
                     AUX1_INPUT_PIN,     AUX2_INPUT_PIN);

/* TODO: enumify? */
size_t flight_mode_index = 0;

static void print_channels(channels_t receiver_in) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
        Serial.print(receiver_in[index]);
        Serial.print("\t");
    }
    Serial.println();
}

extern "C" int main(void) {
    Serial.begin(9600);

    delay(3000);

    pinMode(LED_PIN, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);

    uint64_t interval = 500;
    uint64_t previous = millis();
    while (!receiver.has_signal()) {
        uint64_t current = millis();
        while (current - previous < interval) {
            current = millis();
        }
        Serial.println("No receiver signal! Waiting.");
    }
    Serial.println("Receiver signal detected, continuing.");

    init_mpu6050();

    init_watchdog();

    PIDController roll_controller_rate = PIDController (1.0, 0.0, 0.0, 12.0, 200.0);
    PIDController roll_controller_stbl = PIDController (1.0, 0.0, 0.0, 12.0, 200.0);

    PIDController pitch_controller_rate = PIDController(1.0, 0.0, 0.0, 12.0, 200.0);
    PIDController pitch_controller_stbl = PIDController(1.0, 0.0, 0.0, 12.0, 200.0);

    PIDController yaw_controller_rate = PIDController  (1.0, 0.0, 0.0, 12.0, 200.0);

    Output out_mixer_left(SERVO, LEFT_SERVO_PIN);
    out_mixer_left
         .set_throttle_volume(1.0)
        ->set_roll_volume(1.0)
        ->set_yaw_volume(-0.1);

    Output out_mixer_right(SERVO, RIGHT_SERVO_PIN);
    out_mixer_right
         .set_throttle_volume(1.0)
        ->set_roll_volume(-1.0)
        ->set_yaw_volume(0.1);

    Output out_mixer_front(SERVO, FRONT_SERVO_PIN);
    out_mixer_front
         .set_throttle_volume(1.0)
        ->set_pitch_volume(1.0)
        ->set_yaw_volume(-0.1);

    Output out_mixer_back(SERVO, BACK_SERVO_PIN);
    out_mixer_back
         .set_throttle_volume(1.0)
        ->set_pitch_volume(-1.0)
        ->set_yaw_volume(0.1);

    out_mixer_left.shut_off();
    out_mixer_right.shut_off();
    out_mixer_front.shut_off();
    out_mixer_back.shut_off();

    /* Flight loop */
    while (true) {
        receiver.update(receiver_in);

        // print_channels(receiver_in);

        notime(update_abs_angles(&attitude));

        notime(update_angular_rates(&angular_rate));

        switch (state) {
            case DISARMING:
                Serial.println("Disarming!");
                if (!disarming_input(receiver_in)) {
                    Serial.println("Disarming interrupted! Arming again.");
                    state = ARMED;
                    break;
                } else {
                    state = disarming_complete() ? DISARMED : DISARMING;
                    if (state == DISARMED) {
                        Serial.println("Release the hold!");
                        while (disarming_input(receiver_in)) {
                            out_mixer_left.shut_off();
                            out_mixer_right.shut_off();
                            out_mixer_front.shut_off();
                            out_mixer_back.shut_off();

                            receiver.update(receiver_in);
                            update_abs_angles(&attitude);
                            update_angular_rates(&angular_rate);
                            feed_the_dog();
                            delay(10);
                        }
                        Serial.println("DISARMING COMPLETE!");
                        break;
                    }
                }

                Serial.println("Proceeding to 'ARMED' state actions from 'DISARMING'");
                /* Keep disarming, but stay armed (no break) */

            case ARMED:
                pid_output_roll_stbl = roll_controller_stbl.  compute(micros(), attitude.roll, receiver_in[ROLL_CHANNEL]);

                pid_output_roll_rate = roll_controller_rate.  compute(micros(), angular_rate.roll, -15 * pid_output_roll_stbl);


                pid_output_pitch_stbl = pitch_controller_stbl.compute(micros(), attitude.pitch, receiver_in[PITCH_CHANNEL]);

                pid_output_pitch_rate = pitch_controller_rate.compute(micros(), angular_rate.pitch, -15 * pid_output_pitch_stbl);

                /* Yaw needs rate only - yaw stick controls rate of rotation, there is no fixed reference */
                pid_output_yaw_rate = yaw_controller_rate.    compute(micros(), angular_rate.yaw, receiver_in[YAW_CHANNEL]);

                out_mixer_left. apply(receiver_in[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                out_mixer_right.apply(receiver_in[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                out_mixer_front.apply(receiver_in[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                out_mixer_back. apply(receiver_in[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);

#define DEBUG_COL
#ifdef DEBUG_COL
                Serial.print("thr:");
                Serial.print(receiver_in[THROTTLE_CHANNEL]);
                Serial.print("\tsetp:");
                Serial.print(pid_output_roll_stbl);
                Serial.print("\tr-angl:");
                Serial.print(attitude.roll);
                Serial.print("\tr-p-out:");
                Serial.print(pid_output_roll_stbl);
                Serial.print("\tr-p_rate-out:");
                Serial.println(pid_output_roll_rate);
#endif

                if (state != DISARMING && disarming_input(receiver_in)) {
                    Serial.println("Initializing Disarm!");
                    state = DISARMING;
                    disarm_init();
                }
                break;

            case ARMING:
                Serial.println("Arming!");
                if (!arming_input(receiver_in)) {
                    Serial.println("Arming interrupted! Disarming again.");
                    state = DISARMED;
                    break;
                } else {
                    state = arming_complete() ? ARMED : ARMING;
                    if (state == ARMED) {
                        Serial.println("Release the hold!");
                        while (arming_input(receiver_in)) {
                            /* Still don't fire the motors up */
                            out_mixer_left.shut_off();
                            out_mixer_right.shut_off();
                            out_mixer_front.shut_off();
                            out_mixer_back.shut_off();

                            receiver.update(receiver_in);
                            update_abs_angles(&attitude);
                            update_angular_rates(&angular_rate);
                            feed_the_dog();
                            delay(10);
                        }
                        Serial.println("ARMING COMPLETE!");
                        break;
                    }
                }
                Serial.println("Proceeding to 'DISARMED' state actions from 'ARMING'");
                /* Keep arming, but stay disarmed (no break) */

            case DISARMED:
                out_mixer_left.shut_off();
                out_mixer_right.shut_off();
                out_mixer_front.shut_off();
                out_mixer_back.shut_off();

                if (state != ARMING && arming_input(receiver_in)) {
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
