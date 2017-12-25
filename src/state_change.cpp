#include "../teensy3/Arduino.h"
#include "../teensy3/WProgram.h"

#include "../libraries/Servo/Servo.h"

#include "../interface/state.h"
#include "../interface/watchdog.h"
#include "../interface/receiver.h"
#include "../interface/imu.h"
#include "../interface/settings.h"

#define channels_within_threshold(threshold) \
    receiver_in[THROTTLE_CHANNEL] < threshold && \
    receiver_in[YAW_CHANNEL] < threshold && \
    receiver_in[PITCH_CHANNEL] < threshold && \
    receiver_in[ROLL_CHANNEL] < threshold \

extern uint16_t receiver_in[NUM_CHANNELS];

extern Servo left_ppm;
extern Servo right_ppm;

static bool disarm_started = false;
static uint32_t since_disarm_start;
static uint32_t disarm_start_millis;

static bool arm_started = false;
static uint32_t since_arm_start;
static uint32_t arm_start_millis;

axis_t ignored_data_imu = { 0, 0, 0 };
channels_t ignored_data_rx;

bool check_disarm_status() {
    if (channels_within_threshold(DISARM_THRESHOLD)) {
        if (disarm_started) {
            since_disarm_start = millis() - disarm_start_millis;
            if (since_disarm_start > DISARM_TIMEOUT) {
                disarm_started = false;
                since_disarm_start = 0;
                disarm_start_millis = 0;
                disarm();

                left_ppm.writeMicroseconds(1000);
                right_ppm.writeMicroseconds(1000);

                while (channels_within_threshold(DISARM_THRESHOLD)) {
                    feed_the_dog();
                    read_receiver(&ignored_data_rx);
                    read_abs_angles(&ignored_data_imu);
                    left_ppm.writeMicroseconds(1000);
                    right_ppm.writeMicroseconds(1000);
                    delayMicroseconds(5);
                }
                Serial.println("disarmed!");
                return true;
            }
        } else {
            disarm_started = true;
            disarm_start_millis = millis();
            Serial.println("disarming started!");
        }
    } else {
        disarm_started = false;
    }
    return false;
}

bool check_arm_status() {
    if (channels_within_threshold(ARM_THRESHOLD)) {
        if (arm_started) {
            since_arm_start = millis() - arm_start_millis;
            if (since_arm_start > ARM_TIMEOUT) {
                arm_started = false;
                since_arm_start = 0;
                arm_start_millis = 0;
                arm();
                while (channels_within_threshold(ARM_THRESHOLD)) {
                    feed_the_dog();
                    read_receiver(&ignored_data_rx);
                    read_abs_angles(&ignored_data_imu);
                    delayMicroseconds(5);
                }
                Serial.println("armed!");
                return true;
            }
        } else {
            arm_started = true;
            arm_start_millis = millis();
            Serial.println("arming started!");
        }
    } else {
        arm_started = false;
    }
    return false;
}