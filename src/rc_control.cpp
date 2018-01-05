#include "Arduino.h"
#include "WProgram.h"

#include "../interface/rc_control.h"

/*
   ————————————————————————————————————————————————————
   ———           SERVO GLOBAL VARIABLES             ———
   ————————————————————————————————————————————————————
*/

/* Arm ESC's with a long low pulse */

void arm_ESC(Servo *left, Servo *right, Servo *front, Servo *back) {
    Serial.println("Initialising ESCs: 1000ms pulse");
    left->writeMicroseconds(1000);
    right->writeMicroseconds(1000);
    front->writeMicroseconds(1000);
    back->writeMicroseconds(1000);
    delay(1000);
    Serial.println("Initialised ESCs");
}

