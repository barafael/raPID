#include "../include/Blinker.h"

Blinker *blinker_instance = nullptr;

void update_blink() {
    for (size_t index = 0; index < blinker_instance->PIN_COUNT; index++) {
        char symbol = (blinker_instance->patterns[index])
            [blinker_instance->indices[index]];
        /* assuming strings in patterns are initialized
         * at least as emptystring */
        if (symbol == '\0') {
            blinker_instance->indices[index] = 0;
            symbol = (blinker_instance->patterns[index])[0];
        } else {
            blinker_instance->indices[index]++;
            symbol = (blinker_instance->patterns[index])
                [blinker_instance->indices[index]];
        }
        if (symbol == '1') {
            digitalWrite(blinker_instance->pins[index], HIGH);
        } else {
            digitalWrite(blinker_instance->pins[index], LOW);
        }
    }
}

Blinker::Blinker(uint32_t _update_interval_ms, std::initializer_list<uint8_t> _pins)
    : update_interval_ms (_update_interval_ms)
    , pins { _pins } {
    blinker_instance = this;

    PIN_COUNT = _pins.size();

    for (size_t index = 0; index < PIN_COUNT; index++) {
        pinMode(pins[index], OUTPUT);
        patterns[index] = "0";
    }

    blink_interval_timer.begin(update_blink, _update_interval_ms);
}

void Blinker::set_blink_pattern(uint8_t pin, char *pattern) {
    for (std::vector<uint8_t>::size_type index = 0; index != pins.size(); index++) {
        if (pins[index] == pin) {
            patterns[index] = pattern;
            indices[index] = 0;
            return;
        }
    }
    pins.push_back(pin);
    patterns.push_back(pattern);
    indices.push_back(0);
}

