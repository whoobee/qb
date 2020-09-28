
#ifndef __SCH_MANAGER__
#define __SCH_MANAGER__

#include <HeliOS_Arduino.h>
#include "batt_mon.h"

/* Task frequency definition */
#define T_DEBUG_FREQ     (1000000u)  // 1s
#define T_ROS_FREQ       (1000000u)  // 1s
#define T_BATT_MON_FREQ  (1000000u)  // 1s
#define T_STATE_MON_FREQ (500000u)   // 500ms

/* Scheduler enternal func */
extern void SchMan_Init(void);
extern void SchMan_Spin(void);

extern BatteryMonitor* SchMan_GetBattMonInstance(unsigned int instance);

#endif /* __SCH_MANAGER__*/