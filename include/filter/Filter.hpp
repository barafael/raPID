#ifndef FILTER_H
#define FILTER_H

class Filter {
    public:
    virtual float next(float value) = 0;
};

#endif // FILTER_H
