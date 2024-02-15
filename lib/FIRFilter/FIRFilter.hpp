#pragma once

class FIRFilter
{
private:
    double *coefficients;
    int order;
    double *buffer;
    int bufferIndex = 0;

public:
    FIRFilter(double *coefficients, int order);
    double update(double input);
};