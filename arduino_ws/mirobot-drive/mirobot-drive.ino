/*
 * MiroBot Driver
 * Motor control and telemetry
 */

/* Threading include */
#include <Thread.h>

#define E_TRUE             (1)
#define E_FALSE            (0)

#define EN_TELEMETRY_DATA  E_FALSE

/* ROS include */
#include <ros.h>
#include <std_msgs/String.h>
#include <mirobot_driver/wheel_control.h>
#include <mirobot_driver/wheel_telemetry.h>
#include <mirobot_driver/bot_telemetry.h>

/* Actuators include */
#include "MeMegaPi.h"
#include <Wire.h>

/***********************/
/* Thread definitions  */
/***********************/
Thread pubThread = Thread();


/***********************/
/* ROS definitions     */
/***********************/
ros::NodeHandle  nh;

/* ROS Subscriber */
void WheelDataCbk( const mirobot_driver::wheel_control& wheel_data);
ros::Subscriber<mirobot_driver::wheel_control> sub("wheel_ctrl", WheelDataCbk );

/* ROS Publisher */
void PubCbk(void);

#if (EN_TELEMETRY_DATA == E_TRUE)
mirobot_driver::bot_telemetry bot_tele;
ros::Publisher bot_telemetry("bot_telemetry", &bot_tele);

mirobot_driver::wheel_telemetry wheel_tele;
ros::Publisher wheel_telemetry("wheel_telemetry", &wheel_tele);
#endif  /* #if (EN_TELEMETRY_DATA == E_TRUE) */

/************************/
/* Makeblock definitions*/
/************************/
#if (EN_TELEMETRY_DATA == E_TRUE)
MeGyro gyro;

int32_t gyroX = 0;
int32_t gyroY = 0;
int32_t gyroZ = 0;
int32_t accX  = 0;
int32_t accY  = 0;
int32_t accZ  = 0;

int32_t gyroRight = 0;
int32_t gyroLeft  = 0;
#endif  /* (EN_TELEMETRY_DATA == E_TRUE) */

/* Wheel SLOT definition */
#define FRONT_LEFT_WHEEL  SLOT3
#define REAR_LEFT_WHEEL   SLOT1
#define FRONT_RIGHT_WHEEL SLOT4
#define REAR_RIGHT_WHEEL  SLOT2
/* Left Side Motor Definition */
MeEncoderOnBoard frontLeftWheelMotor(FRONT_LEFT_WHEEL);
MeEncoderOnBoard rearLeftWheelMotor(REAR_LEFT_WHEEL);
/* Right Side Motor Definition */
MeEncoderOnBoard frontRightWheelMotor(FRONT_RIGHT_WHEEL);
MeEncoderOnBoard rearRightWheelMotor(REAR_RIGHT_WHEEL);


int16_t rightWheelsSpeed = 0;
int16_t leftWheelsSpeed  = 0;

typedef enum
{
  e_wheel_front_left  = 0,
  e_wheel_rear_left   = 1,
  e_wheel_front_right = 2,
  e_wheel_rear_right  = 3,
  e_wheels_left       = 4,
  e_wheels_right      = 5
}e_wheel_id_t;

/***********************/
/* Function definitions*/
/***********************/

/* Encoder Interrupt Handlers */
void isr_process_wheel_front_left(void)
{
  if(digitalRead(frontLeftWheelMotor.getPortB()) == 0)
  {
    frontLeftWheelMotor.pulsePosMinus();
  }
  else
  {
    frontLeftWheelMotor.pulsePosPlus();;
  }
}

void isr_process_wheel_rear_left(void)
{
  if(digitalRead(rearLeftWheelMotor.getPortB()) == 0)
  {
    rearLeftWheelMotor.pulsePosMinus();
  }
  else
  {
    rearLeftWheelMotor.pulsePosPlus();
  }
}

void isr_process_wheel_front_right(void)
{
  if(digitalRead(frontRightWheelMotor.getPortB()) == 0)
  {
    frontRightWheelMotor.pulsePosMinus();
  }
  else
  {
    frontRightWheelMotor.pulsePosPlus();
  }
}

void isr_process_wheel_rear_right(void)
{
  if(digitalRead(rearRightWheelMotor.getPortB()) == 0)
  {
    rearRightWheelMotor.pulsePosMinus();
  }
  else
  {
    rearRightWheelMotor.pulsePosPlus();
  }
}

/* Wheel Control */
void WheelsInit(void)
{
  attachInterrupt(frontLeftWheelMotor.getIntNum(),  isr_process_wheel_front_left,  RISING);
  attachInterrupt(rearLeftWheelMotor.getIntNum(),   isr_process_wheel_rear_left,   RISING);
  attachInterrupt(frontRightWheelMotor.getIntNum(), isr_process_wheel_front_right, RISING);
  attachInterrupt(rearRightWheelMotor.getIntNum(),  isr_process_wheel_rear_right,  RISING);

  frontLeftWheelMotor.setPulse(7);
  rearLeftWheelMotor.setPulse(7);
  frontRightWheelMotor.setPulse(7);
  rearRightWheelMotor.setPulse(7);
  
  frontLeftWheelMotor.setRatio(35);
  rearLeftWheelMotor.setRatio(35);
  frontRightWheelMotor.setRatio(35);
  rearRightWheelMotor.setRatio(35);
  
  frontLeftWheelMotor.setPosPid(1.8,0,0.5);
  rearLeftWheelMotor.setPosPid(1.8,0,0.5);
  frontRightWheelMotor.setPosPid(1.8,0,0.5);
  rearRightWheelMotor.setPosPid(1.8,0,0.5);
  
  frontLeftWheelMotor.setSpeedPid(0.18,0,0);
  rearLeftWheelMotor.setSpeedPid(0.18,0,0);
  frontRightWheelMotor.setSpeedPid(0.18,0,0);
  rearRightWheelMotor.setSpeedPid(0.18,0,0);
}

void WheelMove(e_wheel_id_t wheelId, int16_t wheelSpeed)
{
  switch(wheelId)
  {
    case e_wheel_front_left:
    frontLeftWheelMotor.runSpeed(-wheelSpeed);
    break;
    case e_wheel_rear_left:
    rearLeftWheelMotor.runSpeed(-wheelSpeed);
    break;
    case e_wheel_front_right:
    frontRightWheelMotor.runSpeed(-wheelSpeed);
    break;
    case e_wheel_rear_right:
    rearRightWheelMotor.runSpeed(-wheelSpeed);
    break;
    case e_wheels_left:
    frontLeftWheelMotor.runSpeed(-wheelSpeed);
    rearLeftWheelMotor.runSpeed(-wheelSpeed);
    break;
    case e_wheels_right:
    frontRightWheelMotor.runSpeed(-wheelSpeed);
    rearRightWheelMotor.runSpeed(-wheelSpeed);
    break;
    default:
    break;
  }
}

void WheelProcess(void)
{
  frontLeftWheelMotor.loop();
  rearLeftWheelMotor.loop();
  frontRightWheelMotor.loop();
  rearRightWheelMotor.loop();  
}


/* Subscriber Cbk */
void WheelDataCbk( const mirobot_driver::wheel_control& wheel_data)
{
  char    tmpbuf [4];
  int16_t dir = 1;
  
  if(wheel_data.dir_r == 0)
  {
    dir = 1;
  }
  else
  {
    dir = -1;
  }
  rightWheelsSpeed = (dir * (wheel_data.speed_r*2));

  if(wheel_data.dir_l == 0)
  {
    dir = 1;
  }
  else
  {
    dir = -1;
  }
  leftWheelsSpeed  = (dir * (wheel_data.speed_l*2));

  sprintf (tmpbuf, "%03i", (wheel_data.speed_r*2));
  nh.loginfo("RightMotor = ");
  nh.loginfo(tmpbuf);
  sprintf (tmpbuf, "%03i", (wheel_data.speed_l*2));
  nh.loginfo("LeftMotor = ");
  nh.loginfo(tmpbuf);

  WheelMove(e_wheels_left, leftWheelsSpeed);
  WheelMove(e_wheels_right, rightWheelsSpeed);
}

/* pubThread Cbk*/
void PubCbk(void)
{
#if (EN_TELEMETRY_DATA == E_TRUE)
  bot_tele.id = 0;
  bot_tele.gyro_x = gyroX;
  bot_tele.gyro_y = gyroY;
  bot_tele.gyro_z = gyroZ;
  bot_tele.acc_x  = accX;
  bot_tele.acc_y  = accY;
  bot_tele.acc_z  = accZ;
  bot_telemetry.publish( &bot_tele );

  wheel_tele.speed_r = gyroRight;
  wheel_tele.speed_l = gyroLeft;
  wheel_telemetry.publish( &wheel_tele );
#endif  /* #if (EN_TELEMETRY_DATA == E_TRUE) */
}

/* Init function */
void setup()
{
#if (EN_TELEMETRY_DATA == E_TRUE)
  gyro.begin();
#endif  /* #if (EN_TELEMETRY_DATA == E_TRUE) */
  
  WheelsInit();

  pubThread.onRun(PubCbk);
  pubThread.setInterval(500);
  
  nh.initNode();
#if (EN_TELEMETRY_DATA == E_TRUE)
  nh.advertise(bot_telemetry);
#endif
  nh.subscribe(sub);
}

/* Main handler */
void loop()
{
#if (EN_TELEMETRY_DATA == E_TRUE)
  gyro.update();
  gyroX = gyro.getAngleX();
  gyroY = gyro.getAngleY();
  gyroZ = gyro.getAngleZ();
#endif  /* #if (EN_TELEMETRY_DATA == E_TRUE) */  
  // checks if the publisher thread should run
  if(pubThread.shouldRun())
  {
    pubThread.run();
  }

  WheelProcess();

  nh.spinOnce();
  delay(100);
}


/* EOF */
