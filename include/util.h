#ifndef UTIL_H
#define UTIL_H

#define clamp(value, low, high) \
    ((value) = \
    ((value) < (low)  ? (low) : \
    ((value) > (high) ? (high) : (value))))

#endif
