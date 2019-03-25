#include "../include/util.h"

/*@ requires low < high;
    ensures low <= \result <= high;
    assigns \nothing;
    behavior clamped:
      assumes value < low || value > high;
      ensures value < low ==> \result == low;
      ensures value > high ==> \result == high;
    behavior notClamped:
      assumes low <= value <= high;
      ensures \result == \old(value);
    disjoint behaviors clamped, notClamped;
    complete behaviors clamped, notClamped;
*/
int use_clamp(int value, const int low, const int high) {
    mock_clamp(value, low, high);
    return value;
}
