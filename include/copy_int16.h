#ifndef COPY_INT16_H
#define COPY_INT16_H

#include <stddef.h>
#include <stdint.h>

/*@
    predicate IsValidRange_int16{L}(int16_t *a, integer n) =
    \at(0 <= n && \valid(a + (0 .. n - 1)), L);
*/

/*@
    predicate IsEqual_int16{A, B}(int16_t *a, integer n, int16_t *b) =
    \forall integer i; 0 <= i < n ==> \at(*(a + i), A) ≡ \at(*(b + i), B);
*/

/*@
    requires IsValidRange_int16(a, n);
    requires IsValidRange_int16(b, n);
    requires \separated(a + (0 .. n - 1), b + (0 .. n - 1));
    ensures IsEqual_int16{Here, Old}(\old(a), \old(n), \old(a));
    ensures IsEqual_int16{Here, Here}(\old(a), \old(n), \old(b));
    assigns *(b + (0 .. n - 1));
*/
void copy_int16(int16_t const *a, int16_t *b, size_t n);

#endif // COPY_INT16_H
