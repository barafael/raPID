#include "../include/util.h"

/*@ requires low < high;
    ensures low <= \result <= high;
    assigns \nothing;
*/
int use_clamp(int value, const int low, const int high) {
    clamp(value, low, high);
    return value;
}
