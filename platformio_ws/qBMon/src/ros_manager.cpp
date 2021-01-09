
#include "ros_manager.h"
#include "sch_manager.h"
#include "batt_mon.h"
#include "state_mon.h"

#include "qb_ani/qb_ani_status.h"
#include "qb_ani/qb_ani_request.h"

using namespace ros;
using namespace qb_mon;
using namespace qb_ani;

NodeHandle nh;

void qb_battery_state_callback( const std_msgs::Int32& _state);
void qb_ani_request_callback( const qb_ani_request& _request);

qb_batt_status batt_status = qb_batt_status();
Publisher qb_battery_state_pub("qb_battery_state", &batt_status);
Subscriber<std_msgs::Int32> qb_battery_state_sub("qb_state", qb_battery_state_callback);

qb_ani_status ani_status = qb_ani_status();
qb_ani_request ani_request = qb_ani_request();
Publisher qb_ani_state_pub("qb_ani_state", &ani_status);
Subscriber<qb_ani_request> qb_ani_request_sub("qb_ani_request", qb_ani_request_callback);

bool qb_ani_request_received = false;

void RosMan_Init(void)
{
   // Initialize the ROS node.
   nh.initNode();
   nh.subscribe(qb_battery_state_sub);
   nh.advertise(qb_battery_state_pub);

   nh.subscribe(qb_ani_request_sub);
   nh.advertise(qb_ani_state_pub);

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

void qb_battery_state_callback( const std_msgs::Int32& _state)
{
   StateMon_SetState((unsigned char)_state.data);

   Serial.print("Received state: ");
   Serial.println((unsigned char)_state.data);
}

void qb_ani_request_callback( const qb_ani_request& _request)
{
   qb_ani_request_received = true;

   ani_request.request_id = _request.request_id;
   ani_request.rotation_angle[0] = _request.rotation_angle[0];
   ani_request.rotation_angle[1] = _request.rotation_angle[1];
   ani_request.rotation_angle[2] = _request.rotation_angle[2];
   ani_request.rotation_speed[0] = _request.rotation_speed[0];
   ani_request.rotation_speed[1] = _request.rotation_speed[1];
   ani_request.rotation_speed[2] = _request.rotation_speed[2];
   ani_request.recess_travel     = _request.recess_travel;
   ani_request.recess_speed      = _request.recess_speed;


   Serial.print("Received request: (id)");
   Serial.print(_request.request_id);
   Serial.print(" : (rot_ang)[");
   Serial.print(_request.rotation_angle[0]);
   Serial.print(", ");
   Serial.print(_request.rotation_angle[1]);
   Serial.print(", ");
   Serial.print(_request.rotation_angle[2]);
   Serial.print("] : (rot_speed)[");
   Serial.print(_request.rotation_speed[0]);
   Serial.print(", ");
   Serial.print(_request.rotation_speed[1]);
   Serial.print(", ");
   Serial.print(_request.rotation_speed[2]);
   Serial.print("] : (rec_trav)]");
   Serial.print(_request.recess_travel);
   Serial.print("] : (rec_speed)[");
   Serial.print(_request.recess_speed);
   Serial.println("]");
}

void taskRos(int id_)
{
   // Publish data to ROSSERIAL.
   RosMan_UpdateBattData();
   qb_battery_state_pub.publish( &batt_status);

   if(qb_ani_request_received)
   {
      ani_status.request_id = ani_request.request_id;
      ani_status.status = ani_status.ANI_STATUS_RUNNIG;
      qb_ani_state_pub.publish( &ani_status);

      qb_ani_request_received = false;
   }

   nh.spinOnce();
}