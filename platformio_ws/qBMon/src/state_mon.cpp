
#include "state_mon.h"


unsigned char sm_state = STATE_MON_OFF_STATE;
static unsigned int expire_cnt = STATE_MON_EXPIRE_CNT_VAL;

static void StateMon_SetOffState(void);
static void StateMon_SetStartingState(void);
static void StateMon_SetReadyState(void);
static void StateMon_SetRunningState(void);
static void StateMon_SetRecoveryState(void);
static void StateMon_SetErrorState(void);

/* Initialization func */
void StateMon_Init(void)
{
    pinMode(STATE_MON_RED_LED_PIN, OUTPUT);
    pinMode(STATE_MON_GREEN_LED_PIN, OUTPUT);
    pinMode(STATE_MON_BLUE_LED_PIN, OUTPUT);

    digitalWrite(STATE_MON_RED_LED_PIN, HIGH);
    digitalWrite(STATE_MON_GREEN_LED_PIN, HIGH);
    digitalWrite(STATE_MON_BLUE_LED_PIN, HIGH);
}

/* OFF State - LED = SOLID RED */
static void StateMon_SetOffState(void)
{
    digitalWrite(STATE_MON_RED_LED_PIN, LOW);
    digitalWrite(STATE_MON_GREEN_LED_PIN, HIGH);
    digitalWrite(STATE_MON_BLUE_LED_PIN, HIGH);
}

/* OS ON State - LED = SOLID BLUE */
static void StateMon_SetOsOnState(void)
{
    digitalWrite(STATE_MON_RED_LED_PIN, HIGH);
    digitalWrite(STATE_MON_GREEN_LED_PIN, HIGH);
    digitalWrite(STATE_MON_BLUE_LED_PIN, LOW);
}

/* STARTING State - LED = INTERMITENT BLUE */
static void StateMon_SetStartingState(void)
{
    static unsigned char intermitent_blue = 0;

    /* toggle blue LED */
    intermitent_blue = (!intermitent_blue) & (0x01);
    digitalWrite(STATE_MON_RED_LED_PIN, HIGH);
    digitalWrite(STATE_MON_GREEN_LED_PIN, HIGH);
    digitalWrite(STATE_MON_BLUE_LED_PIN, intermitent_blue);
}

/* READY State - LED = SOLID GREEN */
static void StateMon_SetReadyState(void)
{
    digitalWrite(STATE_MON_RED_LED_PIN, HIGH);
    digitalWrite(STATE_MON_GREEN_LED_PIN, LOW);
    digitalWrite(STATE_MON_BLUE_LED_PIN, HIGH);
}

/* RUNNING State - LED = INTERMITENT GREEN */
static void StateMon_SetRunningState(void)
{
    static unsigned char intermitent_green = 0;

    /* toggle green LED */
    intermitent_green = (!intermitent_green) & (0x01);
    digitalWrite(STATE_MON_RED_LED_PIN, HIGH);
    digitalWrite(STATE_MON_GREEN_LED_PIN, intermitent_green);
    digitalWrite(STATE_MON_BLUE_LED_PIN, HIGH);
}

/* RECOVERY State - LED = INTERMITENT GREEN */
static void StateMon_SetRecoveryState(void)
{
    static unsigned char intermitent_green = 0;

    /* toggle green LED */
    intermitent_green = (!intermitent_green) & (0x01);
    digitalWrite(STATE_MON_RED_LED_PIN, HIGH);
    digitalWrite(STATE_MON_GREEN_LED_PIN, intermitent_green);
    digitalWrite(STATE_MON_BLUE_LED_PIN, HIGH);
}

/* ERROR State - LED = INTERMITENT RED */
static void StateMon_SetErrorState(void)
{
    static unsigned char intermitent_red = 0;

    /* toggle red LED */
    intermitent_red = (!intermitent_red) & (0x01);
    digitalWrite(STATE_MON_RED_LED_PIN, intermitent_red);
    digitalWrite(STATE_MON_GREEN_LED_PIN, HIGH);
    digitalWrite(STATE_MON_BLUE_LED_PIN, HIGH);
}

/* Get last set state */
unsigned char StateMon_GetState(void)
{
    return sm_state;
}

/* Set State Mon current state */
void StateMon_SetState(unsigned char _state)
{
    expire_cnt = STATE_MON_EXPIRE_CNT_VAL;
    sm_state = _state;
}

/* Task handler */
void taskStateMon(int id_)
{
    expire_cnt--;
    if(expire_cnt <= 0)
    {
        sm_state = STATE_MON_OFF_STATE;
    }
    
    switch(StateMon_GetState())
    {
        case STATE_MON_OFF_STATE:
            StateMon_SetOffState();
            break;
        case STATE_MON_OS_ON_STATE:
            StateMon_SetOsOnState();
            break;
        case STATE_MON_STARTING_STATE:
            StateMon_SetStartingState();
            break;
        case STATE_MON_READY_STATE:
            StateMon_SetReadyState();
            break;
        case STATE_MON_RUNNING_STATE:
            StateMon_SetRunningState();
            break;
        case STATE_MON_RECOVERY_STATE:
            StateMon_SetRecoveryState();
            break;
        case STATE_MON_ERROR_STATE:
            StateMon_SetErrorState();
            break;
    }
}