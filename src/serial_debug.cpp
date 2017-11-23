#include <Arduino.h>
#include "WProgram.h"

#include "../include/settings.h"
#include "../include/pins.h"

/*
   ————————————————————————————————————————————————————
   ———             SERIAL DEBUG OUTPUT              ———
   ————————————————————————————————————————————————————
*/

extern float yaw_pitch_roll[3];
extern int16_t attitude[3];
extern uint16_t receiver_in[NUM_CHANNELS];
extern int16_t gyro_axis[3];

void print_yaw_pitch_roll() {
    Serial.print(F("yaw_pitch_roll\t"));
    Serial.print(yaw_pitch_roll[YAW_ANGLE]);
    Serial.print(F("\t"));
    Serial.print(yaw_pitch_roll[PITCH_ANGLE]);
    Serial.print(F("\t"));
    Serial.println(yaw_pitch_roll[ROLL_ANGLE]);
}

void print_attitude() {
    Serial.print(F("Attitude\t"));
    Serial.print(attitude[YAW_ANGLE]);
    Serial.print(F("\t"));
    Serial.print(attitude[PITCH_ANGLE]);
    Serial.print(F("\t"));
    Serial.println(attitude[ROLL_ANGLE]);
}

void print_receiver() {
    Serial.print(receiver_in[THROTTLE_CHANNEL]);
    Serial.print(F("\t"));
    Serial.print(receiver_in[ROLL_CHANNEL]);
    Serial.print(F("\t"));
    Serial.print(receiver_in[PITCH_CHANNEL]);
    Serial.print(F("\t"));
    Serial.println(receiver_in[YAW_CHANNEL]);
}

void print_angular_rates() {
    Serial.print(gyro_axis[ROLL_RATE]);
    Serial.print("\t");
    Serial.print(gyro_axis[PITCH_RATE]);
    Serial.print("\t");
    Serial.println(gyro_axis[YAW_RATE]);
}
