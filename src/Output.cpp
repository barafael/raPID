#include "../include/Output.h"

Output::Output(const uint8_t pin)
    : pin   { pin }
    , mixer { 0.0, 0.0, 0.0, 0.0 } {}

Output::Output(const uint8_t pin,
        float throttle_volume,
        float roll_volume, float pitch_volume, float yaw_volume)
    : pin   { pin }
    , mixer { throttle_volume, roll_volume, pitch_volume, yaw_volume } {}

