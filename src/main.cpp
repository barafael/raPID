#include <Arduino.h>

#include <stdint.h>

//#define WATCHDOG

#include "../include/output/FastPWMOutput.h"
#include "../include/imu/axis.hpp"
#include "../include/pid/PIDParams.h"
#include "../include/pid/PIDController.h"
#include "../include/receiver/PWMReceiver.hpp"
#include "../include/ArmingState.hpp"
#include "../include/error_blink.h"
#include "../include/pins.h"
#include "../include/imu/SENtralIMU.hpp"
#include "../include/settings.h"

#ifdef WATCHDOG
#include "../include/Watchdog.h"
#endif

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
state_t state = DISARMED;

/* Scaled yaw_pitch_roll to [0, 1000] */
//axis_t attitude = { 0, 0, 0 };
float attitude[3] = { 0 };

/* Angular Rate */
//axis_t angular_rates = { 0, 0, 0 };
float angular_rates[3] = { 0 };

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

static void print_velocity(float *velocity) {
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

    FastPWMOutput_t temporary = fast_out_init(5  , 1.0 , -1.0 , -1.0 , 1.0 , false);

    uint16_t counter =0;
    while (true) {
        fast_out_apply(&temporary, counter, 0,0, 0);
        delay(500);
        counter++;
    }

    static bool blink_state = false;

    delay(1000);

    channels_t offsets = { -1000, -1500, -1500, -1500, -1500, -1500 };

    PWMReceiver receiver(THROTTLE_INPUT_PIN, ROLL_INPUT_PIN,
                         PITCH_INPUT_PIN,    YAW_INPUT_PIN,
                         AUX1_INPUT_PIN,     AUX2_INPUT_PIN,
                         offsets);
  
    //PPMReceiver receiver(23, offsets);

    while (!receiver.has_signal()) {
        delay(500);
        Serial.println(F("No receiver signal! Waiting."));
    }

    Serial.println(F("Receiver signal detected, continuing."));

    pid_controller_t roll_controller_stbl = pid_controller_init( 2.0 , 0.0 , 0.0 , 12.0 , 400.0);
    pid_controller_t roll_controller_rate = pid_controller_init( 0.65 , 0.0 , 0.0 , 12.0 , 400.0);

    pid_controller_t pitch_controller_stbl = pid_controller_init( 2.0 , 0.0 , 0.0 , 12.0 , 400.0);
    pid_controller_t pitch_controller_rate = pid_controller_init( 0.65 , 0.0 , 0.0 , 12.0 , 400.0);

    pid_controller_t yaw_controller_rate =   pid_controller_init( 1.5 , 0.0 , 0.0 , 12.0 , 400.0);

    FastPWMOutput_t back_left_out_mixer   = fast_out_init(LEFT_SERVO_PIN  , 1.0 , -1.0 , -1.0 , 1.0 , true);
    FastPWMOutput_t back_right_out_mixer  = fast_out_init(RIGHT_SERVO_PIN , 1.0 , 1.0  , -1.0 , -1.0, true);
    FastPWMOutput_t front_left_out_mixer  = fast_out_init(FRONT_SERVO_PIN , 1.0 , -1.0 , 1.0  , -1.0, true);
    FastPWMOutput_t front_right_out_mixer = fast_out_init(BACK_SERVO_PIN  , 1.0 , 1.0  , 1.0  , 1.0 , true);

    fast_out_shutoff(&back_left_out_mixer);
    fast_out_shutoff(&back_right_out_mixer);
    fast_out_shutoff(&front_left_out_mixer);
    fast_out_shutoff(&front_right_out_mixer);

    pid_set_enabled(&roll_controller_stbl, true);
    pid_set_enabled(&roll_controller_rate, true);

    pid_set_enabled(&pitch_controller_stbl, true);
    pid_set_enabled(&pitch_controller_rate, true);

    pid_set_enabled(&yaw_controller_rate, true);

    SENtralIMU sentral;

#ifdef WATCHDOG
    watchdog_init();
#endif

    ArmingState arming_state(channels);

    /* Flight loop */
    while(true) {
        receiver.update(channels);
        //print_channels(channels);

        sentral.update_attitude(attitude);
        //print_attitude(attitude);

        sentral.update_angular_rates(angular_rates);
        //print_velocity(angular_rates);

        switch (arming_state.get_state()) {
            case ARMED:
                pid_output_roll_stbl = pid_compute(&roll_controller_stbl, attitude[ROLL_AXIS], channels[ROLL_CHANNEL]);

                pid_output_roll_rate = pid_compute(&roll_controller_rate, angular_rates[ROLL_AXIS], -pid_output_roll_stbl);

                //pitch_controller_stbl.set_p((channels[ROLL_CHANNEL] + 500) * (15.0 / 1000));
                //pitch_controller_stbl.set_i((channels[YAW_CHANNEL]  + 500) * (5.0  / 1000));

                pid_output_pitch_stbl = pid_compute(&pitch_controller_stbl, attitude[PITCH_AXIS], channels[PITCH_CHANNEL] / 10);

                pid_output_pitch_rate = pid_compute(&pitch_controller_rate, angular_rates[PITCH_AXIS], -pid_output_pitch_stbl);

                /* Yaw needs rate only - yaw stick controls rate of rotation, there is no fixed reference */
                pid_output_yaw_rate = pid_compute(&yaw_controller_rate, angular_rates[YAW_AXIS], channels[YAW_CHANNEL]);


                fast_out_apply(&back_left_out_mixer,   channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                fast_out_apply(&back_right_out_mixer,  channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                fast_out_apply(&front_left_out_mixer,  channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                fast_out_apply(&front_right_out_mixer, channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);

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

                break;

            case DISARMED:
                fast_out_shutoff(&back_left_out_mixer);
                fast_out_shutoff(&back_right_out_mixer);
                fast_out_shutoff(&front_left_out_mixer);
                fast_out_shutoff(&front_right_out_mixer);
                break;

            default:
                Serial.println(F("Unimplemented state! Will disarm."));
                state = DISARMED;
                break;
        }

        /* Blink LED to indicate activity */
        blink_state = !blink_state;
        digitalWrite(LED_PIN, blink_state);
#ifdef WATCHDOG
        watchdog_feed();
#endif
    }
}
