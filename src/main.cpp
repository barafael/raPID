#include <Arduino.h>
#include "WProgram.h"

#include <stdint.h>

#include "Servo.h"
#include "I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#include "error_handling.h"
#include "settings.h"
#include "pins.h"
#include "serial_debug.h"
#include "read_receiver.h"
#include "imu.h"
#include "pid.h"


/*
   ——————————————————————————————————————————————
   ———             HARDWARE SETUP             ———
   ——————————————————————————————————————————————

     MPU6050 Breakout ----- Teensy 3.2
     3.3V ----------------- 3.3V
     GND ------------------ GND
     SDA ------------------ A4/pin 18
     SCL ------------------ A5/pin 19
     INT ------------------ Digital Pin 2

   PPM from RC RX go to pins 9, 10, 11, 12 (see pins.h)
   Output PPM to ESC's: pins 20, 21
*/


static bool blink_state = false;


/* Yaw/Pitch/Roll container and gravity vector
 * [yaw, pitch, roll]
 */
extern float yaw_pitch_roll[3];

/* Scaled yaw_pitch_roll to [0, 1000]
 * [yaw, pitch, roll]
 */
extern int16_t attitude[3];

/* Angular Rates
 * [yaw_rate, pitch_rate, roll_rate]
 */
extern int16_t gyro_axis[3];


extern float pid_output_roll;

/*
   ————————————————————————————————————————————————————
   ———           SERVO GLOBAL VARIABLES             ———
   ————————————————————————————————————————————————————
*/

Servo left_ppm;
Servo right_ppm;
int16_t left_throttle;
int16_t right_throttle;
int16_t throttle;

/* Arm ESC's with a long low pulse */

void arm_ESC() {
    serial_println("Initialising ESCs: 1000ms pulse");
    left_ppm.writeMicroseconds(1000);
    right_ppm.writeMicroseconds(1000);
    delay(1000);
    serial_println("Initialised ESCs");
}


extern uint16_t receiver_in[NUM_CHANNELS];


void inline watchdog_init() {
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);
    /* Enable WDG */
    WDOG_STCTRLH = 0x0001;
    /* The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to. */
    WDOG_TOVALL = 200;
    WDOG_TOVALH = 0;
    /* This sets prescale clock so that the watchdog timer ticks at 1kHZ instead of the default 1kHZ/4 = 200 HZ */
    WDOG_PRESC = 4;
}

void inline feed_the_dog() {
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}


extern "C" int main(void) {
    pinMode(LED_PIN, OUTPUT);
    pinMode(DEBUG_PIN, OUTPUT);

    serial_begin(9600);

    init_rc_interrupts();

    left_ppm.attach(LEFT_SERVO_PIN);
    right_ppm.attach(RIGHT_SERVO_PIN);

    arm_ESC();

    init_MPU6050();

    watchdog_init();

    while(1) {
        //digitalWrite(DEBUG_PIN, HIGH);
        read_angular_rates();
        //digitalWrite(DEBUG_PIN, LOW);

        read_receiver();

        calculate_PID_absolute(receiver_in[ROLL_CHANNEL], attitude[ROLL_ANGLE], gyro_axis[ROLL_RATE]);

        //print_receiver();

        /* wait for MPU interrupt or extra packet(s) available */
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpu_interrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data

        digitalWrite(DEBUG_PIN, HIGH);
        read_abs_angles();
        digitalWrite(DEBUG_PIN, LOW);

        /*
        int value = (gyro_axis[ROLL_RATE] + 2000) * (255.0/4000.0);
        analogWrite(DEBUG_PIN, value);
        */

        throttle = receiver_in[THROTTLE_CHANNEL] + 10;

        left_throttle  = throttle + pid_output_roll;
        right_throttle = throttle - pid_output_roll;

        left_throttle = left_throttle < 0 ? 0 : left_throttle;
        right_throttle = right_throttle < 0 ? 0 : right_throttle;

        left_throttle = left_throttle > 1000 ? 1000 : left_throttle;
        right_throttle = right_throttle > 1000 ? 1000 : right_throttle;

        #define DEBUG_COL
#ifdef DEBUG_COL
        serial_print("thr:");
        serial_print(throttle);
        serial_print("\tsetp:");
        serial_print(receiver_in[ROLL_CHANNEL]);
        serial_print("\tr-angl:");
        serial_print(attitude[ROLL_ANGLE]);
        serial_print("\tleft:");
        serial_print(left_throttle);
        serial_print("\tright:");
        serial_print(right_throttle);
        serial_print("\tr-p-out:");
        serial_println(pid_output_roll);
#endif

        left_ppm.writeMicroseconds(left_throttle + 1000);
        right_ppm.writeMicroseconds(right_throttle + 1000);

        /* Blink LED to indicate activity */
        blink_state = !blink_state;
        digitalWrite(LED_PIN, blink_state);
        feed_the_dog();
    }
}



/*
   ————————————————————————————————————————————————————
   ———             SERIAL DEBUG OUTPUT              ———
   ————————————————————————————————————————————————————
*/

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
            serial_print(F("1"));
        } else {
            serial_print(F("0"));
        }

        --num_places;
        if (((num_places % 4) == 0) && (num_places != 0)) {
            serial_print("_");
        }
    }
    serial_println();
}

void print_all(int num, ...) {
    char* line;
    va_list argList;
    va_start(argList, num);

    for(; num; num--) {
        line = va_arg(argList, char*);
        Serial.print("line: ");
        Serial.println(line);
    }
    va_end(argList);
}

