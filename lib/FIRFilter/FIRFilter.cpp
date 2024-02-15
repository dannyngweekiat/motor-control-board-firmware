#include "FIRFilter.hpp"

FIRFilter::FIRFilter(double *coefficients, int order)
{
    this->coefficients = coefficients;
    this->order = order;
    this->buffer = new double[order];
    for (int i = 0; i < order; i++)
        this->buffer[i] = 0;
}

double FIRFilter::update(double input)
{
    this->buffer[this->bufferIndex] = input;
    this->bufferIndex = (this->bufferIndex + 1) % this->order;
    
    double output = 0;
    for (int i = 0; i < this->order; i++)
        output += this->coefficients[i] * this->buffer[(this->bufferIndex + i) % this->order];

    return output;
}