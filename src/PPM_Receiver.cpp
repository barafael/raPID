#include "Arduino.h"
#include "WProgram.h"

#include "../include/PPM_Receiver.h"

PPMReceiver::PPMReceiver(uint8_t _input_pin) {
    
    input_pin = _input_pin;
    input_ppm.begin(input_pin);
    
    delay(10);
}

const void PPMReceiver::update(channels_t channels) {
    int num = input_ppm.available();
    if (num > 0) {
        count = count + 1;
        for (size_t index = 0; index < NUM_CHANNELS; index++) {
            float val = input_ppm.read(index + 1);
            channels[index] = (uint16_t) val;
            if (channels[index] < 1000) channels[index] = 1000;
            if (channels[index] > 2000) channels[index] = 2000;
            channels[index] -= 1000;
        }
    }
}

const bool PPMReceiver::has_signal() {
    return true;
}
