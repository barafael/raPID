#include "copy_bool.h"

void copy_bool(const bool* a, bool* b, size_t n) {
    /*@
      // <= n because at end of last iteration, i == n
      loop invariant 0 <= i <= n;
      loop invariant IsEqual_bool{Here, Here}(a, i, b);
      loop assigns i, b[0 .. n - 1];
      loop variant n - i;
      */
    for (size_t i = 0; i < n; ++i) {
        b[i] = a[i];
    }
}
