/*
 * MiroBot Driver
 * Motor control and telemetry
 */

/* Threading include */
#include <Thread.h>

/* ROS include */
#include <ros.h>
#include <std_msgs/String.h>
#include <mirobot_driver/wheel_control.h>

/* Actuators include */
#include "MeMegaPi.h"

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
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";


/************************/
/* Makeblock definitions*/
/************************/
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
  char tmpbuf [4];
  
  rightMotorSpeed = wheel_data.speed_r*2;
  leftMotorSpeed  = wheel_data.speed_l*2;

  sprintf (tmpbuf, "%03i", rightMotorSpeed);
  nh.loginfo("RightMotor = ");
  nh.loginfo(tmpbuf);
  sprintf (tmpbuf, "%03i", leftMotorSpeed);
  nh.loginfo("LeftMotor = ");
  nh.loginfo(tmpbuf);
}

/* pubThread Cbk*/
void PubCbk(void)
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
}

/* Init function */
void setup()
{
  pubThread.onRun(PubCbk);
  pubThread.setInterval(500);
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

/* Main handler */
void loop()
{
  // checks if the publisher thread should run
  if(pubThread.shouldRun())
  {
    pubThread.run();
  }

  rightMotor.run(-rightMotorSpeed);   /* value: between -255 and 255. */
  leftMotor.run(leftMotorSpeed);      /* value: between -255 and 255. */
  
  nh.spinOnce();
  delay(1);
}


/* EOF */
