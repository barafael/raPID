#include "../include/util.h"

#include <limits.h>
#include <stdint.h>

/*@ 
    requires value > INT16_MIN;
    ensures \result >= 0;
    assigns \nothing;
*/
int16_t use_abs(int16_t value) {
    abs(value);
    return value;
}
