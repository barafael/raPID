#include "ArduinoMock.h"

int wire_requestFrom(int devAddr, int size);

void wire_beginTransmission(int device);

void wire_write(int address);

int wire_read();

int wire_endTransmission();
int wire_endTransmission_arg(int arg);

int wire_available();
void wire_begin(int, int, int, int, int);
