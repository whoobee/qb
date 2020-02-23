#pragma once

#include <ESP32Encoder.h>

const boolean LEFT PROGMEM = false;
const boolean RIGHT PROGMEM = true;

const boolean ZERO_TO_2PI PROGMEM = false;      // 0 to +2*Pi
const boolean PLUS_MINUS_PI	PROGMEM = true;     // -Pi to +Pi

class MagneticEncoder
{
private:
/* Methods */
  float CalculateAngle(signed int);
	float normalize(float);

/* Encoder */
  ESP32Encoder *encoder;
  signed int last_encoder_cnt = 0;
	boolean angleMode;
	boolean position;
	signed int initial_angle;
  unsigned char _digitalPinA;
  unsigned char _digitalPinB;
  unsigned int _piSteps;
  volatile unsigned long encoderTickCount = 0;


public:
/* Methods */
	MagneticEncoder(unsigned char, unsigned char, unsigned int, boolean);
	MagneticEncoder(unsigned char, unsigned char, unsigned int, boolean, boolean);
  float GetAngle(void);
  signed int GetRawData(void);
};

MagneticEncoder::MagneticEncoder(unsigned char digitalPinA, unsigned char digitalPinB, 
  unsigned int piSteps, boolean position)
  : MagneticEncoder::MagneticEncoder(digitalPinA, digitalPinB, position, piSteps, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(unsigned char digitalPinA, unsigned char digitalPinB, 
  unsigned int piSteps, boolean position, boolean mode)
  :angleMode(mode)
{
  _piSteps = piSteps;
  _digitalPinA = digitalPinA;
  _digitalPinB = digitalPinB;

  encoder = new ESP32Encoder();

  encoder->useInternalWeakPullResistors=true;
  encoder->clearCount();
  encoder->attachHalfQuad(_digitalPinA, _digitalPinB);

  initial_angle = CalculateAngle(GetRawData());
}


signed int MagneticEncoder::GetRawData(void)
{
  return encoder->getCount();
}


float MagneticEncoder::GetAngle() 
{
	// Change sign of the sensor reading
	unsigned char k = 1;
	//this->position == LEFT? k = -1 : k = 1;

	float current_angle = k * CalculateAngle(GetRawData());

	return normalize(current_angle - initial_angle);
}


float MagneticEncoder::CalculateAngle(signed int tickCount)
{
  // M_PI defined in math.h for newer versions
	return tickCount * ((float)2*M_PI / _piSteps);
}


float MagneticEncoder::normalize(float angle) 
{
  if (this->angleMode == PLUS_MINUS_PI) angle += M_PI;
	angle = fmod(angle, 2*PI);
  if (angle < 0) angle += 2*PI;
  if (this->angleMode == PLUS_MINUS_PI) angle -= M_PI;

  return angle;
}
