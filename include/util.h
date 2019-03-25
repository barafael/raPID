#ifndef UTIL_H
#define UTIL_H

#define mock_abs(value) \
    (value) = (value) >= 0 ? (value) : (-(value))

#define mock_clamp(value, low, high) \
    ((value) = \
    ((value) < (low)  ? (low) : \
    ((value) > (high) ? (high) : (value))))

#endif // UTIL_H
