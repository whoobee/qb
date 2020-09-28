
#ifndef __STATE_MON__
#define __STATE_MON__

#include "Arduino.h"

#define STATE_MON_EXPIRE_CNT_VAL (10u)

#define STATE_MON_RED_LED_PIN   (2u)
#define STATE_MON_GREEN_LED_PIN (3u)
#define STATE_MON_BLUE_LED_PIN  (4u)

#define STATE_MON_OFF_STATE      (0u)
#define STATE_MON_OS_ON_STATE    (1u)
#define STATE_MON_STARTING_STATE (2u)
#define STATE_MON_READY_STATE    (3u)
#define STATE_MON_RUNNING_STATE  (4u)
#define STATE_MON_RECOVERY_STATE (5u)
#define STATE_MON_ERROR_STATE    (6u)

extern void StateMon_Init(void);
extern void taskStateMon(int id_);
extern unsigned char StateMon_GetState(void);
extern void StateMon_SetState(unsigned char _state);

#endif /* __STATE_MON__*/