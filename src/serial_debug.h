#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

/*
   ————————————————————————————————————————————————————
   ———             SERIAL DEBUG OUTPUT              ———
   ————————————————————————————————————————————————————
*/

void print_all(int num, ...);
void print_angular_rates();
void print_attitude();
void print_binary(int value, int num_places);
void print_receiver();
void print_yaw_pitch_roll();

#endif // SERIAL_DEBUG_H
