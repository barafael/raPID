#ifndef VECTOR_3_H
#define VECTOR_3_H

template <typename T> struct Vector3 {
    T x;
    T y;
    T z;

    public:
    Vector3(T x, T y, T z)
        : x(x)
        , y(y)
        , z(z) {};

    T &operator[](size_t index) {
        switch(index % 3) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
        }
    }
};

#endif // VECTOR_3_H

