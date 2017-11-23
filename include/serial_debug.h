#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

/*
   ————————————————————————————————————————————————————
   ———             SERIAL DEBUG OUTPUT              ———
   ————————————————————————————————————————————————————
*/

#define ENABLE_DEBUG_PRINT

#ifdef ENABLE_DEBUG_PRINT
    #define serial_println(a); (Serial.println(a)); Serial.flush();
    #define serial_print(a); (Serial.print(a)); Serial.flush();
    #define serial_begin(a); (Serial.begin(a)); Serial.flush();
#else
    #define serial_println(a);
    #define serial_print(a);
    #define serial_begin(a);
#endif

void print_all(int num, ...);
void print_angular_rates();
void print_attitude();
void print_binary(int value, int num_places);
void print_receiver();
void print_yaw_pitch_roll();
void print_all(int num, ...);
void print_binary(int value, int num_places);

#endif // SERIAL_DEBUG_H
