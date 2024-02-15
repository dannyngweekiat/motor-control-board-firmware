#pragma once

class IncrementalPID
{
private:
    /* data */
    double Kp;                // PID Kp value
    double Ki;                // PID Ki value
    double Kd;                // PID Kd value
    double previous_err1 = 0; // e(k-1)
    double previous_err2 = 0; // e(k-2)
    double last_output = 0;   // PID output in last control period

public:
    IncrementalPID(double Kp, double Ki, double Kd);

    double calculate(double error);
    void updateParameters(double Kp, double Ki, double Kd);
    void reset();
};
