
/* Debug Configuration */
#include "Debug.h"

/* HW Configuration */
#include "ESP32HW.h"

/* Network Driver */
#include "WiFiHW.h"

/* Motor Driver */
#include "DCMotor.h"

/* Encoder Driver */
#include "MagneticEncoder.h"

/* ROS */
#include <ros.h>
#include <std_msgs/Float32.h>

/* Scheduler */
#include <Thread.h>

/***************/
/******WIFI*****/
/***************/
char ipAddress[4] = {192,168,0,67};
WiFiHW *wifi;


/**************/
/******ROS*****/
/**************/
ros::NodeHandle nh;
IPAddress *server;
std_msgs::Float32 pubLeftWheelAngleMsg, pubRightWheelAngleMsg;
ros::Publisher *pubLeftWheelAngle, *pubRightWheelAngle;

void LeftWheelSpeedCbk(const std_msgs::Float32& msg);
void RightWheelSpeedCbk(const std_msgs::Float32& msg);
ros::Subscriber<std_msgs::Float32> subLeftWheelSpeed("mirobot/left_wheel_vel", &LeftWheelSpeedCbk);
ros::Subscriber<std_msgs::Float32> subRightWheelSpeed("mirobot/right_wheel_vel", &RightWheelSpeedCbk);


/******************/
/******ENCODER*****/
/******************/
MagneticEncoder *lEncoder, *rEncoder;
float lNewPosition = 0, lOldPosition = 0;
float rNewPosition = 0, rOldPosition = 0;


/****************/
/******MOTOR*****/
/****************/
DCMotor *lController, *rController;;
signed int speedLeft, speedRight;


/********************/
/******SCHEDULER*****/
/********************/
#define OTA_THREAD_INTERVAL_MS     (2000u)
#define ROS_THREAD_INTERVAL_MS     (10u)
#define ENCODER_THREAD_INTERVAL_MS (10u)
#define MOTOR_THREAD_INTERVAL_MS   (100u)

Thread OTAThread     = Thread();
Thread ROSThread     = Thread();
Thread ENCODERThread = Thread();
Thread MOTORThread   = Thread();

void MOTORThreadCbk(void);
void ENCODERThreadCbk(void);
void OTAThreadCbk(void);
void ROSThreadCbk(void);

void SchedulerInit(void)
{
  OTAThread.onRun(OTAThreadCbk);
  OTAThread.setInterval(OTA_THREAD_INTERVAL_MS);

  ROSThread.onRun(ROSThreadCbk);
  ROSThread.setInterval(ROS_THREAD_INTERVAL_MS);

  ENCODERThread.onRun(ENCODERThreadCbk);
  ENCODERThread.setInterval(ENCODER_THREAD_INTERVAL_MS);

  MOTORThread.onRun(MOTORThreadCbk);
  MOTORThread.setInterval(MOTOR_THREAD_INTERVAL_MS);
}

void SchedulerSpin(void)
{
  if(OTAThread.shouldRun()) OTAThread.run();
  if(ROSThread.shouldRun()) ROSThread.run();
  if(ENCODERThread.shouldRun()) ENCODERThread.run();
  if(MOTORThread.shouldRun()) MOTORThread.run();
}

void MOTORThreadCbk(void)
{
  lController->control(speedLeft);
  rController->control(speedRight);
}

float ENCODERCalculateVel(float currentPosition, float *newPosition, float *oldPosition, float timeInterval, float calibrationOffset)
{
  float ret_vel;
  float pos;

  *oldPosition = *newPosition;
  *newPosition = currentPosition;
  
  if(*oldPosition <= *newPosition)
  {
    pos = *newPosition - *oldPosition;
  }
  else
  {
    pos = *newPosition + (2*PI) - *oldPosition;
  }
  
  if(pos != 0.0f)
  {
    ret_vel = (pos/timeInterval) - calibrationOffset;
  }
  else
  {
    ret_vel = 0.0f;
  }

  return ret_vel;
}


void ENCODERThreadCbk(void)
{
  float pos;

  //pubLeftWheelAngleMsg.data = ENCODERCalculateVel(lEncoder->GetAngle(), &lNewPosition, &lOldPosition, 0.2f, 0.3f);
  //pubRightWheelAngleMsg.data = ENCODERCalculateVel(rEncoder->GetAngle(), &rNewPosition, &rOldPosition, 0.2f, 0.0f);

  pubRightWheelAngleMsg.data = rEncoder->GetAngle();
  pubLeftWheelAngleMsg.data  = lEncoder->GetAngle();
  
  if(DEBUG)Serial.print(": Left Encoder Angle = ");
  if(DEBUG)Serial.print(pubLeftWheelAngleMsg.data);
  if(DEBUG)Serial.print(" : Right Encoder Angle = ");
  if(DEBUG)Serial.println(pubRightWheelAngleMsg.data);
  if(DEBUG)Serial.println();
}

void OTAThreadCbk(void)
{
  wifi->OTAHandling();
}

void ROSThreadCbk(void)
{
  if (nh.connected()) 
  {
    pubRightWheelAngle->publish(&pubRightWheelAngleMsg);
    pubLeftWheelAngle->publish(&pubLeftWheelAngleMsg);
  }
  else 
  {
    if(DEBUG)Serial.println("Not Connected");
  }

  /* Handle ROS spin */
  nh.spinOnce();
}


/************************/
/******ROS Callbacks*****/
/************************/
void ROSInit(void)
{
  /* Set the connection to rosserial socket server */
  server = new IPAddress(ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);
  nh.getHardware()->setConnection(*server, 11411);
  nh.initNode();

  /* Advertize the Encoder publishers */
  pubLeftWheelAngle = new ros::Publisher("mirobot/left_wheel_angle", &pubLeftWheelAngleMsg);
  pubRightWheelAngle = new ros::Publisher("mirobot/right_wheel_angle", &pubRightWheelAngleMsg);
  nh.advertise(*pubLeftWheelAngle);
  nh.advertise(*pubRightWheelAngle);

  /* Define and subscribe the motor subscribers */
  nh.subscribe(subLeftWheelSpeed);
  nh.subscribe(subRightWheelSpeed);
}

int mapToInt(float val) 
{
  int mVal = 0;
  
  /* map(val, 0.0f, 1.0f, 0u, 255u) is buggy.. therfore do it the old fashion way */
  if (val >= 1.0f) mVal = 255;
  else if (val <= -1) mVal = -255;
  else mVal = (int)(val*255);

  return mVal;
}

void LeftWheelSpeedCbk(const std_msgs::Float32& msg) 
{
  speedLeft = mapToInt(msg.data);
  //if(DEBUG)Serial.print("Left Motor Speed = ");
  //if(DEBUG)Serial.println(speedLeft);
}

void RightWheelSpeedCbk(const std_msgs::Float32& msg) 
{
  speedRight = mapToInt(msg.data);
  //if(DEBUG)Serial.print("Right Motor Speed = ");
  //if(DEBUG)Serial.println(speedRight);
}


/***************/
/******INIT*****/
/***************/
void setup()
{
  if(DEBUG)Serial.begin(115200);
  
  /* Init WiFi an OTA */
  wifi = new WiFiHW("NamNet", "U455h0le00", true);

  /* Init ROS */
  ROSInit();

  SchedulerInit();

  lEncoder = new MagneticEncoder(LW_ENCODER_A, LW_ENCODER_B, (ENCODER_2PI_STEPS/2), LEFT, ZERO_TO_2PI);
  rEncoder = new MagneticEncoder(RW_ENCODER_A, RW_ENCODER_B, (ENCODER_2PI_STEPS/2), RIGHT, ZERO_TO_2PI);

  lController = new DCMotor( IN1, AN1);
  rController = new DCMotor( IN2, AN2);
}


/********************/
/******MAIN LOOP*****/
/********************/
void loop()
{
  SchedulerSpin();
}
