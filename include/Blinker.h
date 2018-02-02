#ifndef BLINKER_H
#define BLINKER_H

#include <vector>

#include "Arduino.h"
#include "pins.h"

class Blinker {
    private:
        uint32_t update_interval_ms;
        IntervalTimer blink_interval_timer;

        size_t PIN_COUNT = 0;

        std::vector<uint8_t> pins;
        std::vector<char*> patterns;
        std::vector<size_t> indices = { 0 };

        friend void update_blink();

    public:
        Blinker(uint32_t _update_interval_ms,
                std::initializer_list<uint8_t> pins);

        void set_blink_pattern(uint8_t pin, char *pattern);
};

#endif // BLINKER_H
