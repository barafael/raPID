#include "../include/copy_bool.h"

void copy_bool(bool const *a, bool *b, size_t n) {
    /*@ loop invariant 0 <= i <= n;
        loop invariant IsEqual_bool{Here, Here}(a, i, b);
        loop assigns i, *(b + (0 .. n - 1));
        loop variant n - i;
     */
    for (size_t i = 0; i < n; ++i) {
        /*@ assert rte: mem_access: \valid(b + i); */
        /*@ assert rte: mem_access: \valid_read(a + i); */
        b[i] = a[i];
    }
}
