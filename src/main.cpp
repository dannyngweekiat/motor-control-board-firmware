#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#include "ICM_20948.h"

#include "Motor.hpp"
#include "DriveWheel.hpp"
#include "IncrementalPID.hpp"
#include "SimpleSerialParser.hpp"

// Hardware Definitions
#define RIGHT_MOTOR_ENCODER 2
#define RIGHT_MOTOR_PWM 9
#define RIGHT_MOTOR_DIRECTION 8
#define LEFT_MOTOR_ENCODER 3
#define LEFT_MOTOR_PWM 11
#define LEFT_MOTOR_DIRECTION 12
#define EMERGENCY_SWITCH 6

// Robot Definitions
#define GEAR_RATIO 100
#define PULSE_PER_REVOLUTION 9
#define MAX_ANGULAR_VECLOCITY 6.28320
#define COMMUNICATION_TIMEOUT_MS 1500

// Robot State
#define ROBOT_IDLE 0
#define ROBOT_RUNNING 1
#define ROBOT_EMERGENCY 2

// Motor Initialization

SAMI363525 *leftMotorPtr;
SAMI363525 *rightMotorPtr;

IncrementalPID *leftPidPtr;
IncrementalPID *rightPidPtr;

DriveWheel *leftWheelPtr;
DriveWheel *rightWheelPtr;

ICM_20948_I2C icm;

void handleLeftMotorEncoderUpdate()
{
  leftMotorPtr->updateEncoder();
}

void handleRightMotorEncoderUpdate()
{
  rightMotorPtr->updateEncoder();
}

bool isEmergency()
{
  return digitalRead(EMERGENCY_SWITCH) == LOW;
}

void getFloatFromEEPROM(int address, double *value, double defaultValue = 0.0)
{
  EEPROM.get(address, *value);
  if (*value == 0.0 || isnan(*value))
  {
    // Set default value
    *value = defaultValue;
    EEPROM.put(address, *value);
  }
}

double kP, kI, kD;

bool updateRobotState(int *state, int updatedState)
{
  if (*state != updatedState)
  {
    *state = updatedState;
    return true;
  }
  return false;
}

void stopRobot()
{
  leftWheelPtr->setVelocity(0.0);
  rightWheelPtr->setVelocity(0.0);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup()
{
  // Motor Initialization
  leftMotorPtr = new SAMI363525(LEFT_MOTOR_ENCODER, LEFT_MOTOR_PWM, LEFT_MOTOR_DIRECTION);
  rightMotorPtr = new SAMI363525(RIGHT_MOTOR_ENCODER, RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIRECTION);

  // Read PID values from EEPROM
  getFloatFromEEPROM(0, &kP, 150.0);
  getFloatFromEEPROM(sizeof(double), &kI, 110.0);
  getFloatFromEEPROM(sizeof(double) * 2, &kD, 0.0);

  leftPidPtr = new IncrementalPID(kP, kI, kD);
  rightPidPtr = new IncrementalPID(kP, kI, kD);
  leftWheelPtr = new DriveWheel(leftPidPtr, leftMotorPtr, GEAR_RATIO, PULSE_PER_REVOLUTION, MAX_ANGULAR_VECLOCITY, MOTOR_FOWARD_DIRECTION_CCW);
  rightWheelPtr = new DriveWheel(rightPidPtr, rightMotorPtr, GEAR_RATIO, PULSE_PER_REVOLUTION, MAX_ANGULAR_VECLOCITY);

  // Hardware initialization
  analogWriteResolution(12);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(EMERGENCY_SWITCH, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_MOTOR_ENCODER), handleLeftMotorEncoderUpdate, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MOTOR_ENCODER), handleRightMotorEncoderUpdate, RISING);

  // Set initial speed
  leftWheelPtr->setVelocity(0.0);
  rightWheelPtr->setVelocity(0.0);
  Serial.begin(500000);

  // Wire Initialization
  Wire.begin();
  Wire.setClock(400000);
  icm.begin(Wire, 0);

  // IMU Initialization
  bool initialized = false;
  int imuRetries = 0;
  while (!initialized)
  {
    icm.begin(Wire, 0);

    if (icm.status != ICM_20948_Stat_Ok)
      delay(500);
    else
      initialized = true;
    if (++imuRetries >= 4)
      break;
  }
  // Startup Delay
  delay(250);
}

void loop()
{
  static SimpleSerialParser serialParser;
  static unsigned long lastUpdate = 0;
  static int ledUpdate = 0;
  static int state = ROBOT_IDLE;
  static bool sendData = false;
  bool serialDataAvailable = serialParser.serialCheck();
  static unsigned long lastCommunication = 0;
  ParsedSerialData data;

  if (isEmergency())
  {
    if (updateRobotState(&state, ROBOT_EMERGENCY))
      Serial.println("E");
    stopRobot(); 
    // Switch Debounce
    delay(50);
  }
  else
  {
    if (state == ROBOT_EMERGENCY)
    {
      if (updateRobotState(&state, ROBOT_IDLE))
        Serial.println("e");
    }
  }

  if (serialDataAvailable)
  {
    lastCommunication = micros();
    data = serialParser.parseSerialData();
    switch (data.command)
    {
    case 's':
      if (updateRobotState(&state, ROBOT_IDLE))
      {
        Serial.println("s");
        stopRobot();
      }
      else
      {
        switch (state)
        {
        case ROBOT_EMERGENCY:
          Serial.println("!s:Robot in emergency state;");
          break;
        case ROBOT_IDLE:
          Serial.println("!s:Robot already idle;");
          break;
        default:
          break;
        }
      }
      break;
    case 'S':
      if (updateRobotState(&state, ROBOT_RUNNING))
      {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("S");
      }
      else
      {
        switch (state)
        {
        case ROBOT_EMERGENCY:
          Serial.println("!s:Robot in emergency state;");
          break;
        case ROBOT_RUNNING:
          Serial.println("!s:Robot already running;");
          break;
        default:
          break;
        }
      }
      break;
    case '.':
      break;
    case 'm':
      if (data.count == 2 && state == ROBOT_RUNNING)
      {
        if (data.data[0].type == 'f' && data.data[1].type == 'f')
        {
          double leftSpeed = data.data[0].floatData;
          double rightSpeed = data.data[1].floatData;
          leftWheelPtr->setVelocity(leftSpeed);
          rightWheelPtr->setVelocity(rightSpeed);
        }
      }
      else
      {
        Serial.println("!s:Robot Idle;");
      }
      break;
    case 'P':
      if (data.count == 3)
      {
        if (data.data[0].type == 'f' && data.data[1].type == 'f' && data.data[2].type == 'f')
        {
          kP = data.data[0].floatData;
          kI = data.data[1].floatData;
          kD = data.data[2].floatData;
          leftPidPtr->updateParameters(kP, kI, kD);
          rightPidPtr->updateParameters(kP, kI, kD);
          Serial.println("p");
        }
      }
      else
      {
        Serial.println("!s:Invalid data;");
      }
      break;
    case 'p':
      Serial.print("pfff:");
      Serial.print(kP, 4);
      Serial.print(";");
      Serial.print(kI, 4);
      Serial.print(";");
      Serial.print(kD, 4);
      Serial.println(";");
      break;
    case 'c':
      EEPROM.put(0, data.data[0].floatData);
      EEPROM.put(sizeof(double), data.data[1].floatData);
      EEPROM.put(sizeof(double) * 2, data.data[2].floatData);
      Serial.println("c");
      break;
    case 'd':
      sendData = true;
      break;
    default:
      Serial.print("!s:Invalid command - ");
      Serial.print(data.command);
      Serial.println(";");
      break;
    }
  }

  // Handle Communication Timeout
  if (micros() - lastCommunication >= COMMUNICATION_TIMEOUT_MS * 1000 && state == ROBOT_RUNNING)
  {
    if (updateRobotState(&state, ROBOT_IDLE))
    {
      Serial.println("t");
      stopRobot();
    }
  }

  // Handle Robot Task - Robot State Machine runs at 20Hz
  if (micros() - lastUpdate >= 50 * 1000)
  {
    lastUpdate = micros();

    auto left = leftWheelPtr->update();
    auto right = rightWheelPtr->update();

    if (icm.dataReady())
      icm.getAGMT();

    if (sendData)
    {
      Serial.print("dffffffffffff:");
      Serial.print(left.angularVelocity, 4);
      Serial.print(";");
      Serial.print(right.angularVelocity, 4);
      Serial.print(";");
      Serial.print(icm.accX(), 2);
      Serial.print(";");
      Serial.print(icm.accY(), 2);
      Serial.print(";");
      Serial.print(icm.accZ(), 2);
      Serial.print(";");
      Serial.flush();
      Serial.print(icm.gyrX(), 2);
      Serial.print(";");
      Serial.print(icm.gyrY(), 2);
      Serial.print(";");
      Serial.print(icm.gyrZ(), 2);
      Serial.print(";");
      Serial.print(icm.magX(), 2);
      Serial.print(";");
      Serial.print(icm.magY(), 2);
      Serial.print(";");
      Serial.print(icm.magZ(), 2);
      Serial.print(";");
      Serial.print(icm.temp(), 2);
      Serial.println(";");
      Serial.flush();
      sendData = false;
    }
  }
}
