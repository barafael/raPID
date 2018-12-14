#ifndef UTIL_H
#define UTIL_H

#define abs(value) \
    (value) = (value) >= 0 ? (value) : (-(value))

#define clamp(value, low, high) \
    ((value) = \
    ((value) < (low)  ? (low) : \
    ((value) > (high) ? (high) : (value))))

#endif // UTIL_H
