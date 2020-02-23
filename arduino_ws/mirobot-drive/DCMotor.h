#pragma once

#include "Debug.h"
#include <analogWrite.h>

class DCMotor {
public:
  DCMotor(int dirPin, int analogPin);
  void control(signed int motorSpeed);

private:
  uint8_t _dirPin, _analogPin;
  int _motorSpeed;
};

DCMotor::DCMotor(int dirPin, int analogPin)
{
  _dirPin = dirPin;
  _analogPin = analogPin;

  pinMode(_dirPin, OUTPUT);
  pinMode(_analogPin, OUTPUT);
}

void DCMotor::control(signed int motorSpeed)
{
   if (motorSpeed >= 0) {
     if (motorSpeed > 100) motorSpeed = 100;
     motorSpeed = motorSpeed * 2.55;
     analogWrite(_analogPin, motorSpeed);
     digitalWrite(_dirPin, LOW);
   }
   else if (motorSpeed < 0) {
     if (motorSpeed < -100) motorSpeed = -100;
     motorSpeed = motorSpeed * -2.55;
     analogWrite(_analogPin, motorSpeed);
     digitalWrite(_dirPin, HIGH);
   }
}
