#ifndef VECTOR_3_H
#define VECTOR_3_H

template <class T> class Vector3 {
    T values[3];

    public:
    Vector3(T x, T y, T z) : values{ x, y, z } {}
    T &operator[](size_t index) {
        return values[index % 3];
    }
};

#endif // VECTOR_3_H
