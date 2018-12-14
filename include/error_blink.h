#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

#include <string.h>

#include "./ArduinoMock.h"

typedef enum {
    DMP_INIT_MEM_LOAD_FAILED,
    DMP_CONF_UPDATES_FAILED,
    DMP_ERROR_UNKNOWN,
    STATE_TIMER_HARDWARE_BUSY
} error_type;

void blink_pattern(const char *pattern);
void error_blink(error_type error, const char *message);

#endif // ERROR_HANDLING_H
