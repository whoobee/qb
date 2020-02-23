#ifndef CYTRON_SMARTDRIVEDUO_h
#define CYTRON_SMARTDRIVEDUO_h

#include "Arduino.h"
#include <analogWrite.h>

class Cytron_SmartDriveDuo
{
  public:
    Cytron_SmartDriveDuo(int in1Pin, int in2Pin, int an1Pin, int an2Pin); // For PWM Independent mode
    void control(signed int motorLeftSpeed, signed int motorRightSpeed);
    
  private:
  	uint8_t _an1Pin, _an2Pin, _in1Pin, _in2Pin, _channel1, _channel2;
   int _motorLSpeed, _motorRSpeed;
};

#endif
