#ifndef COPY_BOOL_H
#define COPY_BOOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*@
    predicate IsValidRange_bool{L}(bool *a, integer n) =
    \at(0 <= n && \valid(a + (0 .. n - 1)), L);
*/

/*@
    predicate IsEqual_bool{A, B}(bool *a, integer n, bool *b) =
    \forall integer i; 0 <= i < n ==> \at(*(a + i), A) ≡ \at(*(b + i), B);
*/

/*@
    requires IsValidRange_bool(a, n);
    requires IsValidRange_bool(b, n);
    requires \separated(a + (0 .. n - 1), b + (0 .. n - 1));
    ensures IsEqual_bool{Here, Old}(\old(a), \old(n), \old(a));
    ensures IsEqual_bool{Here, Here}(\old(a), \old(n), \old(b));
    assigns *(b + (0 .. n - 1));
*/
void copy_bool(bool const *a, bool *b, size_t n);

#endif // COPY_BOOL_H
