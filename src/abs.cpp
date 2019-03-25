#include "../include/util.h"

#include <limits.h>
#include <stdint.h>

/*@ 
    requires value > INT16_MIN;
    ensures \result >= 0;
    assigns \nothing;

    behavior pos:
      assumes value >= 0;
      ensures \result == value;

    behavior neg:
      assumes value < 0;
      ensures \result == -value;

    complete behaviors pos, neg;
    disjoint behaviors pos, neg;
*/
int16_t use_abs(int16_t value) {
    mock_abs(value);
    return value;
}
