#pragma once

#define ANALOG_WRITE_MAX 4095

enum MotorDirection
{
    MOTOR_DIRECTION_CW,
    MOTOR_DIRECTION_CCW
};

class Motor
{
public:
    virtual void setDirection(MotorDirection) = 0;
    virtual MotorDirection getDirection() = 0;

    virtual void setPWM(int) = 0;
    virtual void updateEncoder() = 0;
    virtual int getEncoderDifference() = 0;
    virtual double getEncoderTimeDifference() = 0;
};

class SAMI363525 : public Motor
{
public:
    SAMI363525(int encoderPin, int pwmPin, int directionPin);

    void setDirection(MotorDirection);
    MotorDirection getDirection();

    void setPWM(int);

    void updateEncoder();
    int getEncoderDifference();
    double getEncoderTimeDifference();

private:
    int encoderPin;
    unsigned long lastEncoderTime = 0;
    long timeDifference = 0;
    int pwmPin;
    int directionPin;

    MotorDirection direction;

    int encoder;
    int lastEncoder;
};
