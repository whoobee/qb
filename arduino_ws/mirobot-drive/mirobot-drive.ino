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


MeMegaPiDCMotor rightMotor(PORT1B); /* Right Motor */
MeMegaPiDCMotor leftMotor(PORT2B);  /* Left Motor */
int16_t rightMotorSpeed = 0;
int16_t leftMotorSpeed  = 0;

/***********************/
/* Function definitions*/
/***********************/
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
  rightMotorSpeed = (dir * (wheel_data.speed_r*2));

  if(wheel_data.dir_l == 0)
  {
    dir = 1;
  }
  else
  {
    dir = -1;
  }
  leftMotorSpeed  = (dir * (wheel_data.speed_l*2));

  //sprintf (tmpbuf, "%03i", rightMotorSpeed);
  //nh.loginfo("RightMotor = ");
  //nh.loginfo(tmpbuf);
  //sprintf (tmpbuf, "%03i", leftMotorSpeed);
  //nh.loginfo("LeftMotor = ");
  //nh.loginfo(tmpbuf);
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

  rightMotor.run(-rightMotorSpeed);   /* value: between -255 and 255. */
  leftMotor.run(leftMotorSpeed);      /* value: between -255 and 255. */

  nh.spinOnce();
  delay(100);
}


/* EOF */
