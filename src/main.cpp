#include "Arduino.h"

#include <stdint.h>

#include "../include/output/ESCOutput.hpp"
#include "../include/output/FastPWMOutput.hpp"
#include "../include/output/LEDOutput.hpp"
#include "../include/imu/axis.hpp"
#include "../include/pid/PIDController.hpp"
#include "../include/receiver/PWMReceiver.hpp"
#include "../include/receiver/PPMReceiver.hpp"
#include "../include/ArmingState.hpp"
#include "../include/error_blink.h"
#include "../include/pins.h"
#include "../include/imu/SENtralIMU.hpp"
#include "../include/settings.h"
#include "../include/Watchdog.hpp"

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

   SENtral Breakout ----- Teensy 3.2
   3.3V ----------------- 15
   GND ------------------ 14
   SDA ------------------ A4/pin 18
   SCL ------------------ A5/pin 19
   INT ------------------ Digital Pin 6 (see pins.h)

   See ../include/pins.h for more pin definitions.
   */

/* Default start state */
//state_t state = /*DIS*/ARMED;
state_t state = DISARMED;

/* Scaled yaw_pitch_roll to [0, 1000] */
axis_t attitude = { 0, 0, 0 };

/* Angular Rate */
axis_t angular_rates = { 0, 0, 0 };

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

static void print_velocity(axis_t velocity) {
    for (size_t index = 0; index < 3; index++) {
        Serial.print(velocity[index]);
        Serial.print(F("\t"));
    }
    Serial.println();
}

static void print_velocity_max(axis_t velocity) {
    static int64_t max_velocity = 0;
    for (size_t index = 0; index < 3; index++) {
        if (velocity[index] > max_velocity || velocity[index] < -max_velocity) {
            max_velocity = velocity[index];
            Serial.println((long) max_velocity);
        }
    }
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

    static bool blink_state = false;

    delay(1000);

    channels_t offsets = { -1000, -1500, -1500, -1500, -1500, -1500 };

    PWMReceiver receiver(THROTTLE_INPUT_PIN, ROLL_INPUT_PIN,
                         PITCH_INPUT_PIN,    YAW_INPUT_PIN,
                         AUX1_INPUT_PIN,     AUX2_INPUT_PIN,
                         offsets);
  
    // PPMReceiver receiver(PPM_PIN, offsets);

    while (!receiver.has_signal()) {
        delay(500);
        Serial.println(F("No receiver signal! Waiting."));
    }

    Serial.println(F("Receiver signal detected, continuing."));

    PIDParams<float> roll_param_stbl ( 0.75 , 0.0 , 0.0 , 12.0 , 400.0);
    PIDParams<float> roll_param_rate ( 1.5 , 0.0 , 0.0 , 12.0 , 400.0);

    PIDParams<float> pitch_param_stbl( 0.75 , 0.0 , 0.0 , 12.0 , 400.0);
    PIDParams<float> pitch_param_rate( 1.5 , 0.0 , 0.0 , 12.0 , 400.0);

    PIDParams<float> yaw_param_rate  ( 1.5 , 0.0 , 0.0 , 12.0 , 400.0);

    PIDController<float> roll_controller_stbl(&roll_param_stbl);
    PIDController<float> roll_controller_rate(&roll_param_rate);

    PIDController<float> pitch_controller_stbl(&pitch_param_stbl);
    PIDController<float> pitch_controller_rate(&pitch_param_rate);

    PIDController<float> yaw_controller_rate(&yaw_param_rate);

    FastPWMOutput back_left_out_mixer  (LEFT_SERVO_PIN  , 1.0 , -0.25 , -0.25 , 0.0);
    FastPWMOutput back_right_out_mixer (RIGHT_SERVO_PIN , 1.0 , 0.25  , -0.25 , 0.0);
    FastPWMOutput front_left_out_mixer (FRONT_SERVO_PIN , 1.0 , -0.25 , 0.25  , 0.0);
    FastPWMOutput front_right_out_mixer(BACK_SERVO_PIN  , 1.0 , 0.25  , 0.25  , 0.0);

    back_left_out_mixer  .shut_off();
    back_right_out_mixer .shut_off();
    front_left_out_mixer .shut_off();
    front_right_out_mixer.shut_off();

    roll_controller_stbl.set_enabled(false);
    roll_controller_rate.set_enabled(true);

    pitch_controller_stbl.set_enabled(false);
    pitch_controller_rate.set_enabled(true);

    SENtralIMU sentral;

    Watchdog dog;

    ArmingState arming_state(channels);

    /* Flight loop */
    while (true) {
        receiver.update(channels);
        print_channels(channels);

        sentral.update_attitude(attitude);
        //print_attitude(attitude);

        sentral.update_angular_rates(angular_rates);
        //print_velocity(angular_rates);

        switch (arming_state.get_state()) {
            case ARMED:
                pid_output_roll_stbl = roll_controller_stbl.  compute(attitude[ROLL_AXIS], channels[ROLL_CHANNEL]);

                pid_output_roll_rate = roll_controller_rate.  compute(angular_rates[ROLL_AXIS], pid_output_roll_stbl);

                pid_output_pitch_stbl = pitch_controller_stbl.compute(attitude[PITCH_AXIS], channels[PITCH_CHANNEL]);

                pid_output_pitch_rate = pitch_controller_rate.compute(angular_rates[PITCH_AXIS], pid_output_pitch_stbl);

                /* Yaw needs rate only - yaw stick controls rate of rotation, there is no fixed reference */
                pid_output_yaw_rate = yaw_controller_rate.    compute(angular_rates[YAW_AXIS], channels[YAW_CHANNEL]);

                back_left_out_mixer  .apply(channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                back_right_out_mixer .apply(channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                front_left_out_mixer .apply(channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                front_right_out_mixer.apply(channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);

#define DEBUG_COL
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

                break;

            case DISARMED:
                back_left_out_mixer  .shut_off();
                back_right_out_mixer .shut_off();
                front_left_out_mixer .shut_off();
                front_right_out_mixer.shut_off();

                break;

            default:
                Serial.println(F("Unimplemented state! Will disarm."));
                state = DISARMED;
                break;
        }

        /* Blink LED to indicate activity */
        blink_state = !blink_state;
        digitalWrite(LED_PIN, blink_state);
        dog.feed();
    }
}
