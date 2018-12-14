#include <stddef.h>
#include <stdint.h>

//#define WATCHDOG

#include "../include/ArduinoMock.h"
#include "../include/arming_state.h"
#include "../include/axis.hpp"
#include "../include/error_blink.h"
#include "../include/imu/sentral_imu.hpp"
#include "../include/output/FastPWMOutput.h"
#include "../include/pid/pid_controller.h"
#include "../include/pid/pid_param.h"
#include "../include/pins.h"
#include "../include/receiver/pwm_receiver.h"
#include "../include/settings.h"

#ifdef WATCHDOG
#include "../include/watchdog.h"
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

/* ----------------------------------------------
   ---             HARDWARE SETUP             ---
   ----------------------------------------------

   See ../include/pins.h for pin definitions.  */

int16_t channels[NUM_CHANNELS] = { 0 };
int16_t offsets[NUM_CHANNELS]  = { -1000, -1500, -1500, -1500, -1500, -1500 };

/* Default start state */
arming_state_t state = DISARMED;

const uint64_t SERIAL_WAIT_TIMEOUT = 3000ul;

/* Scaled yaw_pitch_roll to [0, 1000] */
vec3_t attitude;

vec3_t acceleration;
vec3_t angular_velocity;
vec3_t magnetization;

float pid_output_roll_stbl = 0.0;
float pid_output_roll_rate = 0.0;

float pid_output_pitch_stbl = 0.0;
float pid_output_pitch_rate = 0.0;

float pid_output_yaw_rate = 0.0;

static void print_attitude(vec3_t attitude) {
#ifdef USE_SERIAL
    Serial.print(attitude.x);
    Serial.print(F("\t"));
    Serial.print(attitude.y);
    Serial.print(F("\t"));
    Serial.print(attitude.z);
    Serial.print(F("\t"));
    Serial.println();
#endif
}

static void print_velocity(vec3_t angular_velocity) {
#ifdef USE_SERIAL
    Serial.print(angular_velocity.x);
    Serial.print(F("\t"));
    Serial.print(angular_velocity.y);
    Serial.print(F("\t"));
    Serial.print(angular_velocity.z);
    Serial.print(F("\t"));
    Serial.println();
#endif
}

static void print_channels(int16_t channels[NUM_CHANNELS]) {
    for (size_t index = 0; index < NUM_CHANNELS; index++) {
#ifdef USE_SERIAL
        Serial.print(channels[index]);
        Serial.print(F("\t"));
#endif
    }
#ifdef USE_SERIAL
    Serial.println();
#endif
}

extern "C" int main(void) {
#ifdef USE_SERIAL
    Serial.begin(9600);
#endif

    uint64_t serial_wait_start_time = mock_millis();
#ifdef USE_SERIAL
    while (!Serial) {
        if (mock_millis() - serial_wait_start_time > SERIAL_WAIT_TIMEOUT) {
            break;
        }
    }
#endif

    mock_pinMode(LED_PIN, OUTPUT);
    mock_pinMode(DEBUG_PIN, OUTPUT);

    /* Scaled yaw_pitch_roll to [0, 1000] */
    vec3_t attitude;

    // vec3_t acceleration;
    vec3_t angular_velocity;
    // vec3_t magnetization;

    init_arming_state(channels);

    static bool blink_state = false;

    pwm_receiver_init(THROTTLE_INPUT_PIN, ROLL_INPUT_PIN,
                         PITCH_INPUT_PIN, YAW_INPUT_PIN,
                         AUX1_INPUT_PIN,  AUX2_INPUT_PIN,
                         offsets);

    bool receiver_active = false;
    if (receiver_active) {
        while (!has_signal()) {
            mock_delay(500);
#ifdef USE_SERIAL
            Serial.println(F("No receiver signal! Waiting."));
#endif
        }
#ifdef USE_SERIAL
        Serial.println(F("Receiver signal detected, continuing."));
#endif
    }

    pid_controller_t roll_controller_stbl = pid_controller_init(2.0, 0.0, 0.0, 12.0, 400.0);
    pid_controller_t roll_controller_rate = pid_controller_init(0.65, 0.0, 0.0, 12.0, 400.0);

    pid_controller_t pitch_controller_stbl = pid_controller_init(2.0, 0.0, 0.0, 12.0, 400.0);
    pid_controller_t pitch_controller_rate = pid_controller_init(0.65, 0.0, 0.0, 12.0, 400.0);

    pid_controller_t yaw_controller_rate = pid_controller_init(1.5, 0.0, 0.0, 12.0, 400.0);

    FastPWMOutput_t back_left_out_mixer   = fast_out_init(LEFT_SERVO_PIN, 1.0, -1.0, -1.0, 1.0, true);
    FastPWMOutput_t back_right_out_mixer  = fast_out_init(RIGHT_SERVO_PIN, 1.0, 1.0, -1.0, -1.0, true);
    FastPWMOutput_t front_left_out_mixer  = fast_out_init(FRONT_SERVO_PIN, 1.0, -1.0, 1.0, -1.0, true);
    FastPWMOutput_t front_right_out_mixer = fast_out_init(BACK_SERVO_PIN, 1.0, 1.0, 1.0, 1.0, true);

    fast_out_shutoff(&back_left_out_mixer);
    fast_out_shutoff(&back_right_out_mixer);
    fast_out_shutoff(&front_left_out_mixer);
    fast_out_shutoff(&front_right_out_mixer);

    pid_set_enabled(&roll_controller_stbl, true);
    pid_set_enabled(&roll_controller_rate, true);

    pid_set_enabled(&pitch_controller_stbl, true);
    pid_set_enabled(&pitch_controller_rate, true);

    pid_set_enabled(&yaw_controller_rate, true);

    sentral_imu_t imu = init_sentral_imu();

#ifdef WATCHDOG
    init_watchdog();
#endif

    /* Flight loop */
    while (true) {
        receiver_update(channels);
        //print_channels(channels);

        /* For outer control loop */
        attitude = update_attitude(&imu);
        //print_attitude(attitude);

        /* For inner control loop */
        angular_velocity = update_gyroscope(&imu);
        //print_velocity(angular_rates);

        update_arming_state();

        switch (get_arming_state()) {
            case ARMED:
                pid_output_roll_stbl = pid_compute(&roll_controller_stbl, attitude.x, channels[ROLL_CHANNEL]);

                pid_output_roll_rate = pid_compute(&roll_controller_rate, angular_velocity.x, -pid_output_roll_stbl);

                //pitch_controller_stbl.set_p((channels[ROLL_CHANNEL] + 500) * (15.0 / 1000));
                //pitch_controller_stbl.set_i((channels[YAW_CHANNEL]  + 500) * (5.0  / 1000));

                pid_output_pitch_stbl = pid_compute(&pitch_controller_stbl, attitude.y, channels[PITCH_CHANNEL] / 10);

                pid_output_pitch_rate = pid_compute(&pitch_controller_rate, angular_velocity.y, -pid_output_pitch_stbl);

                /* Yaw needs rate only - yaw stick controls rate of rotation, there is no fixed reference */
                pid_output_yaw_rate = pid_compute(&yaw_controller_rate, angular_velocity.z, channels[YAW_CHANNEL]);


                fast_out_apply(&back_left_out_mixer,   channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                fast_out_apply(&back_right_out_mixer,  channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                fast_out_apply(&front_left_out_mixer,  channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);
                fast_out_apply(&front_right_out_mixer, channels[THROTTLE_CHANNEL], pid_output_roll_rate, pid_output_pitch_rate, pid_output_yaw_rate);

#define DEBUG_COL
#ifdef DEBUG_COL
#ifdef USE_SERIAL
                Serial.print(F("setp:"));
                Serial.print(channels[ROLL_CHANNEL]);
                Serial.print(F("\troll-angl:"));
                Serial.print(attitude.x);
                Serial.print(F("\tpid_output_roll_stbl:"));
                Serial.print(pid_output_roll_stbl);
                Serial.print(F("\tpid_output_roll_rate:"));
                Serial.println(pid_output_roll_rate);
#endif
#endif
                break;

            case DISARMED:
                fast_out_shutoff(&back_left_out_mixer);
                fast_out_shutoff(&back_right_out_mixer);
                fast_out_shutoff(&front_left_out_mixer);
                fast_out_shutoff(&front_right_out_mixer);
#define DEBUG_COL
#ifdef DEBUG_COL
#ifdef USE_SERIAL
                Serial.print(F("setp:"));
                Serial.print(channels[ROLL_CHANNEL]);
                Serial.print(F("\troll-angl:"));
                Serial.print(attitude.x);
                Serial.print(F("\tpid_output_roll_stbl:"));
                Serial.print(pid_output_roll_stbl);
                Serial.print(F("\tpid_output_roll_rate:"));
                Serial.println(pid_output_roll_rate);
#endif
#endif
                break;

            default:
#ifdef USE_SERIAL
                Serial.println(F("Unimplemented state!"));
#endif
                //internal_state = INTERNAL_DISARMED;
                break;
        }

        /* Blink LED to indicate activity */
        blink_state = !blink_state;
        mock_digitalWrite(LED_PIN, blink_state);
#ifdef WATCHDOG
        watchdog_feed();
#endif
    }
}
