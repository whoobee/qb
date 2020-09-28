
#ifndef __ROS_MANAGER__
#define __ROS_MANAGER__

#include "Arduino.h"
#define USE_TEENSY_HW_SERIAL
#include <ros.h>
#include <qb_mon/qb_batt_status.h>
#include <std_msgs/Int32.h>

extern void RosMan_Init(void);
extern void taskRos(int id_);
extern void RosMan_UpdateBattData(qb_mon::qb_batt_status battData);

#endif /* __ROS_MANAGER__*/