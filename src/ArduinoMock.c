#include "../include/ArduinoMock.h"

#define FRAMAC

#ifdef FRAMAC

/*@
  ensures \old(previous_millis) < previous_millis;
  assigns previous_millis;
*/
long mock_millis() {
    static unsigned long previous_millis = 0;
    previous_millis = millis();
    return previous_millis;
}

/*@
  ensures \old(previous_micros) < previous_micros;
  assigns previous_micros;
*/
long previous_micros() {
    static unsigned long previous_micros = 0;
    previous_micros = micros();
    return previous_micros;
}

#endif // FRAMAC
