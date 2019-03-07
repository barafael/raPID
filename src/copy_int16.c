#include "../include/copy_int16.h"

void copy_int16(int16_t const *a, int16_t *b, size_t n) {
    /*@ loop invariant 0 <= i <= n;
        loop invariant IsEqual_int16{Here, Here}(a, i, b);
        loop assigns i, *(b + (0 .. n - 1));
        loop variant n - i;
     */
    for (size_t i = 0; i < n; ++i) {
        /*@ assert GLOBAL_undef_behavior: mem_access: \valid(b + i); */
        /*@ assert GLOBAL_undef_behavior: mem_access: \valid_read(a + i); */
        b[i] = a[i];
    }
}
