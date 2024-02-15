#include "DifferentialDrive.hpp"

DifferentialDrive::DifferentialDrive(SAMI363525 *leftMotor, SAMI363525 *rightMotor)
{
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
}