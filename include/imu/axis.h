#ifndef AXIS_H
#define AXIS_H

#include<array>

using axis_index = enum {
    ROLL_AXIS  = 0,
    PITCH_AXIS = 1,
    YAW_AXIS   = 2,
};

using axis_t = std::array<int16_t, 3>;

#endif // AXIS_H

