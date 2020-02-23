/*
Original written by:
  08/05/17  Idris, Cytron Technologies
*/

#include "Cytron_SmartDriveDuo.h"

Cytron_SmartDriveDuo::Cytron_SmartDriveDuo(int in1Pin, int in2Pin, int an1Pin, int an2Pin, int channel1, int channel2)
{
  _an1Pin = an1Pin;
  _an2Pin = an2Pin;
  _in1Pin = in1Pin;
  _in2Pin = in2Pin;
  _channel1 = channel1;
  _channel2 = channel2;

  pinMode(_an1Pin, OUTPUT);
  pinMode(_an2Pin, OUTPUT);
  pinMode(_in1Pin, OUTPUT);
  pinMode(_in2Pin, OUTPUT);
}

void Cytron_SmartDriveDuo::control(signed int motorLSpeed, signed int motorRSpeed)
{
   if (motorLSpeed >= 0) {
     if (motorLSpeed > 100) motorLSpeed = 100;
     motorLSpeed = motorLSpeed * 2.55;
     analogWrite(_an1Pin, motorLSpeed);
     digitalWrite(_in1Pin, LOW);
   }
   else if (motorLSpeed < 0) {
     if (motorLSpeed < -100) motorLSpeed = -100;
     motorLSpeed = motorLSpeed * -2.55;
     analogWrite(_an1Pin, motorLSpeed);
     digitalWrite(_in1Pin, HIGH);
   }
   
   if (motorRSpeed >= 0) {
     if (motorRSpeed > 100) motorRSpeed = 100;
     motorRSpeed = motorRSpeed * 2.55;
     analogWrite(_an2Pin, motorRSpeed);
     digitalWrite(_in2Pin, HIGH);
   }
   else if (motorRSpeed < 0) {
     if (motorRSpeed < -100) motorRSpeed = -100;
     motorRSpeed = motorRSpeed * -2.55;
     analogWrite(_an2Pin, motorRSpeed);
     digitalWrite(_in2Pin, LOW);
   }
}
