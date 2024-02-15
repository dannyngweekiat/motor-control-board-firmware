#include "IncrementalPID.hpp"
#include <Arduino.h>

IncrementalPID::IncrementalPID(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

double IncrementalPID::calculate(double error)
{
    float output = 0;

    /* Calculate the pid control value by increment formula */
    /* du(k) = (e(k)-e(k-1))*Kp + (e(k)-2*e(k-1)+e(k-2))*Kd + e(k)*Ki */
    /* u(k) = du(k) + u(k-1) */
    output = (error - this->previous_err1) * this->Kp +
             (error - 2 * this->previous_err1 + this->previous_err2) * this->Kd +
             error * this->Ki +
             this->last_output;

    /* Update previous error */
    this->previous_err2 = this->previous_err1;
    this->previous_err1 = error;

    /* Update last output */
    this->last_output = output;

    return output;
}

void IncrementalPID::updateParameters(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->reset();
}

void IncrementalPID::reset()
{
    this->previous_err1 = 0;
    this->previous_err2 = 0;
    this->last_output = 0;
}