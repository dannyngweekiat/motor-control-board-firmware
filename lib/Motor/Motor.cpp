#include <Arduino.h>
#include "Motor.hpp"

SAMI363525::SAMI363525(int encoderPin, int pwmPin, int directionPin)
{
    // Hardware Setup
    pinMode(encoderPin, INPUT_PULLUP);
    pinMode(pwmPin, OUTPUT);
    pinMode(directionPin, OUTPUT);

    this->encoderPin = encoderPin;
    this->pwmPin = pwmPin;
    this->directionPin = directionPin;

    // Property Initialization
    this->direction = MOTOR_DIRECTION_CW;
    this->encoder = 0;
    this->lastEncoder = 0;

    // Initial Value
    digitalWrite(this->directionPin, direction);
    analogWrite(this->pwmPin, 0);
}

void SAMI363525::setDirection(MotorDirection direction)
{
    this->direction = direction;
    digitalWrite(this->directionPin, direction);
}

MotorDirection SAMI363525::getDirection()
{
    return this->direction;
}

void SAMI363525::updateEncoder()
{
    auto currentTime = micros();
    this->encoder += this->direction == MOTOR_DIRECTION_CW ? 1 : -1;
    this->timeDifference = (this->direction == MOTOR_DIRECTION_CW ? 1 : -1) * (currentTime - this->lastEncoderTime);
    this->lastEncoderTime = currentTime;
}

int SAMI363525::getEncoderDifference()
{
    int difference = this->encoder - this->lastEncoder;
    this->lastEncoder = this->encoder;
    return difference;
}

double SAMI363525::getEncoderTimeDifference()
{
    return this->timeDifference;
}

void SAMI363525::setPWM(int pwm)
{
    if (pwm > 4096)
        pwm = 4096;
    analogWrite(this->pwmPin, pwm);
}
