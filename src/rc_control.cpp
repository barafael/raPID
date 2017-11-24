#include <Arduino.h>
#include "WProgram.h"

#include "Servo.h"


/*
   ————————————————————————————————————————————————————
   ———           SERVO GLOBAL VARIABLES             ———
   ————————————————————————————————————————————————————
*/

extern Servo left_ppm;
extern Servo right_ppm;

/* Arm ESC's with a long low pulse */

void arm_ESC() {
    Serial.println("Initialising ESCs: 1000ms pulse");
    left_ppm.writeMicroseconds(1000);
    right_ppm.writeMicroseconds(1000);
    delay(1000);
    Serial.println("Initialised ESCs");
}
