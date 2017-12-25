#include "../teensy3/Arduino.h"
#include "../teensy3/WProgram.h"

#include "../interface/settings.h"
#include "../interface/pins.h"

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

void print_binary(int value, int num_places) {
    int mask = 0;

    for (int n = 1; n <= num_places; n++) {
        mask = (mask << 1) | 0x0001;
    }
    /* truncate v to specified number of places */
    value = value & mask;

    while (num_places) {
        if (value & (0x0001 << (num_places - 1))) {
            Serial.print(F("1"));
        } else {
            Serial.print(F("0"));
        }

        --num_places;
        if (((num_places % 4) == 0) && (num_places != 0)) {
            Serial.print("_");
        }
    }
    Serial.println();
}

void print_all(int num, ...) {
    char* line;
    va_list argList;
    va_start(argList, num);

    for (; num; num--) {
        line = va_arg(argList, char*);
        Serial.print("line: ");
        Serial.println(line);
    }
    va_end(argList);
}
