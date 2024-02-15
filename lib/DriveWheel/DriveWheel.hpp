#pragma once
#include "Motor.hpp"
#include "IncrementalPID.hpp"
#include "FIRFilter.hpp"

struct DriveWheelVelocity
{
    double angularVelocity;
    double dt;
};

enum MOTOR_FOWARD_DIRECTION
{
    MOTOR_FOWARD_DIRECTION_CW,
    MOTOR_FOWARD_DIRECTION_CCW
};

class DriveWheel
{
private:
    Motor *motor;
    IncrementalPID *pid;
    double gearRatio;
    int pulsePerRevolution;
    double maxAngularVelocity;
    unsigned long lastUpdate = 0;
    MOTOR_FOWARD_DIRECTION fowardDriveDirection;
    double setPoint = 0;
    int lastDifference = 0;
    double output = 0;
    FIRFilter *filter;

public:
    DriveWheel(IncrementalPID *pid, Motor *motor, double gearRatio,
               int pulsePerRevolution, double maxAngularVelocity,
               MOTOR_FOWARD_DIRECTION motorFowardDirection = MOTOR_FOWARD_DIRECTION_CW);

    DriveWheelVelocity update();
    void setVelocity(double angularVelocity);
};