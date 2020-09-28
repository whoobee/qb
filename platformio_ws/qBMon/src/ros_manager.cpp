
#include "ros_manager.h"
#include "sch_manager.h"
#include "batt_mon.h"
#include "state_mon.h"

using namespace ros;
using namespace qb_mon;

NodeHandle nh;

void qb_state_callback( const std_msgs::Int32& _state);

qb_batt_status batt_status = qb_batt_status();
Publisher qb_battery_state("qb_battery_state", &batt_status);
Subscriber<std_msgs::Int32> sub("qb_state", qb_state_callback);

void RosMan_Init(void)
{
   // Initialize the ROS node.
   nh.initNode();
   nh.subscribe(sub);
   nh.advertise(qb_battery_state);

   // Populate battery parameters.
   batt_status.id = 0;
   batt_status.status = batt_status.STATUS_READY;
   batt_status.type = 0;
   batt_status.capacity = 0;
   batt_status.max_capacity = 5000;
   batt_status.voltage = 0;
   batt_status.serial_number = "qB-Batt-001";
}


static unsigned int batt_id = 0;

void RosMan_UpdateBattData(void)
{
   char serial_number[32];
   
   BatteryMonitor * bMonInstance = SchMan_GetBattMonInstance(batt_id);
   if(bMonInstance != (BatteryMonitor *)0)
   {
      // Populate battery parameters.
      batt_status.id = bMonInstance->GetId();
      batt_status.status = 0;
      batt_status.type = bMonInstance->GetType();
      batt_status.capacity = bMonInstance->GetCapacity();
      batt_status.max_capacity = bMonInstance->GetMaxCapacity();
      batt_status.voltage = bMonInstance->GetVoltage();
      bMonInstance->GetSn().toCharArray(serial_number, bMonInstance->GetSn().length()+1);
      batt_status.serial_number = serial_number;
   }
   if(batt_id == 0)
   {
      batt_id = 1;
   }
   else
   {
      batt_id = 0;
   }
}

void qb_state_callback( const std_msgs::Int32& _state)
{
   StateMon_SetState((unsigned char)_state.data);

   Serial.print("Received state: ");
   Serial.println((unsigned char)_state.data);
}

void taskRos(int id_)
{
   // Publish data to ROSSERIAL.
   RosMan_UpdateBattData();
   qb_battery_state.publish( &batt_status);
   nh.spinOnce();
}