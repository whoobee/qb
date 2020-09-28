
/* Includes */
#include "sch_manager.h"
#include "debug.h"
#include "ros_manager.h"
#include "state_mon.h"

/* Task handlers definition */
int hndlDebug;
int hndlRos;
int hndlBattMon;
int hndlStateMon;

BatteryMonitor battMonA(0, "BATT_A", BatteryMonitor::LI_ION, A0, 33, 44, 5000);
BatteryMonitor battMonB(1, "BATT_B", BatteryMonitor::LI_ION, A1, 33, 44, 5000);

void taskBattMon(int id_)
{
   battMonA.run();
   battMonB.run();
}

BatteryMonitor* SchMan_GetBattMonInstance(unsigned int id)
{
    BatteryMonitor *bMonInstance = (BatteryMonitor*)0;

    switch (id)
    {
      case 0:
         bMonInstance = &battMonA;
         break;
      case 1:
         bMonInstance = &battMonB;
         break;
      default:
         break;
    }

    return bMonInstance;
}

/* Scheduler initialization */
void SchMan_Init(void)
{
    /* Init OS */
    xHeliOSSetup();

    /* Configure debug task */
    hndlDebug = xTaskAdd("tDEBUG", &taskDebug);
    xTaskWait(hndlDebug);
    xTaskSetTimer(hndlDebug, T_DEBUG_FREQ);

    /* Configure ROS task */
    hndlRos = xTaskAdd("tROS", &taskRos);
    xTaskWait(hndlRos);
    xTaskSetTimer(hndlRos, T_ROS_FREQ);

    /* Configure BattMon task */
    int hndlBattMon = xTaskAdd("tBATTMON", &taskBattMon);
    xTaskWait(hndlBattMon);
    xTaskSetTimer(hndlBattMon, T_BATT_MON_FREQ);

    /* Configure StateMon task */
    hndlStateMon = xTaskAdd("tSTATEMON", &taskStateMon);
    xTaskWait(hndlStateMon);
    xTaskSetTimer(hndlStateMon, T_STATE_MON_FREQ);
}

/* Scheduler loop function */
void SchMan_Spin(void)
{
    /* manage OS */
    xHeliOSLoop();
}