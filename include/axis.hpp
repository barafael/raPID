#ifndef AXIS_H
#define AXIS_H

#include<math.h>
#include<stdbool.h>

typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

bool normalize_vec3(vec3_t *vector);

#endif // AXIS_H
