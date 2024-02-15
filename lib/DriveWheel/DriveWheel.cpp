#include <Arduino.h>
#include "DriveWheel.hpp"

double coefficients[] = {
    0.25,
    0.25,
    0.25,
    0.25,
};

DriveWheel::DriveWheel(IncrementalPID *pid, Motor *motor,
                       double gearRatio, int pulsePerRevolution,
                       double maxAngularVelocity, MOTOR_FOWARD_DIRECTION fowardDriveDirection)
{
    this->pid = pid;
    this->motor = motor;
    this->gearRatio = gearRatio;
    this->pulsePerRevolution = pulsePerRevolution;
    this->fowardDriveDirection = fowardDriveDirection;
    this->maxAngularVelocity = maxAngularVelocity;
    this->filter = new FIRFilter(coefficients, 4);
}

DriveWheelVelocity DriveWheel::update()
{
    double timeDifference = (micros() - this->lastUpdate) / 1e6;
    this->lastUpdate = micros();

    // Preparing all needed variables
    double difference = motor->getEncoderDifference();
    // Calculate RPM from encoder
    double angularVelocity = angularVelocity = (2 * PI / (this->pulsePerRevolution * this->gearRatio)) / (motor->getEncoderTimeDifference() / 1000000.0);

    if (this->fowardDriveDirection == MOTOR_FOWARD_DIRECTION_CCW)
        angularVelocity = -angularVelocity;

    // Angular velocity should be 0 if diference is 0 for 2 consequent update
    if (difference == 0 && lastDifference == 0)
        angularVelocity = 0.0;

    angularVelocity = this->filter->update(angularVelocity);

    // Normalized the velocity for PID
    double error = (this->setPoint - angularVelocity);
    double output = this->pid->calculate(error);
    this->output += output;

    this->motor->setDirection(output > 0 ? (this->fowardDriveDirection == MOTOR_DIRECTION_CW ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW)
                                         : (this->fowardDriveDirection == MOTOR_DIRECTION_CCW ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW));
    this->motor->setPWM(fabs(output));
    
    // Reset PID if setPoint is 0.0 and angularVelocity is 0.0
    if (this->setPoint == 0.0 && angularVelocity == 0.0)
    {
        this->output = 0.0;
        this->pid->reset();
    }

    lastDifference = difference;

    return {angularVelocity, timeDifference};
}

void DriveWheel::setVelocity(double velocity)
{
    this->setPoint = velocity;
}