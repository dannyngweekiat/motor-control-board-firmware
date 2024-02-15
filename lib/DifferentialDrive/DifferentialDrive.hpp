#pragma once
#include "Motor.hpp"

class DifferentialDrive
{
private:
    SAMI363525 *leftMotor;
    SAMI363525 *rightMotor;

public:
    DifferentialDrive(SAMI363525 *leftMotor, SAMI363525 *RightMotor);
};
