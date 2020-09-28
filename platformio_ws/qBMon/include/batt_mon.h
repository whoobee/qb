
#ifndef __BATTERY_MON__
#define __BATTERY_MON__

#include "Arduino.h"

#define NUM_OF_READINGS 100               /* Smoothing number of samples */

class BatteryMonitor
{

    private:
        /*********************/
        /* Private Variables */
        /*********************/
        /* Sensor config */
        float K = 0.003225806;            /* volts per ADC unit */
        float SCALING_FACTOR = 18.08;     /* Voltage divider scaling factor (R1 34Kohm; R2 2Kohm) */

        /* BATT config */
        unsigned int ID = 0;
        String SN = "BAT-xxx";
        unsigned int TYPE = 0;
        float MAX_BATT_VOLTAGE = 42;      /* Max voltage of the battery */
        float MIN_BATT_VOLTAGE = 33;      /* Min voltage of the battery */
        float MAX_BATT_CAPACITY = 5000;   /* Max capacity of the battery */
        float MIN_BATT_CAPACITY = 0;      /* Min capacity of the battery */

        /* ADC config */
        unsigned int inputPin = A0; /* ADC channel A0 */

        /* ADC smoothing */
        unsigned int readings[NUM_OF_READINGS];     /* the readings from the analog input */
        unsigned int readIndex = 0;                 /* the index of the current reading */
        float total = 0;                            /* the running total */
        float average = 0;                          /* the average */

        /* Voltage/Capacity */
        float currentBatteryVoltage = 0;
        unsigned int currentBatteryPercent = 0;
        float currentBatteryCapacity = 0;

        /* Debug variables */
        String stringSensorVal;
        String stringSensorVolt;
        String stringBattVolt;
        String stringBattPer;
        String stringBattCapacity;

        /*********************/
        /* Private Functions */
        /*********************/
        float ReadADCVoltage(void);

    public:
      /****************/
      /* Public Enums */
      /****************/
      enum BATT_TYPE
      {
         LI_ION = 0,
         LI_PO,
         NI_MH,
         NI_CD,
         LEAD_ACID
      };
      /********************/
      /* Public Variables */
      /********************/

      /********************/
      /* Public Functions */
      /********************/
      BatteryMonitor(unsigned int _id, String _sn, unsigned int _type, 
         int _adc_in_pin, float _min_voltage, 
         float _max_voltage, float _max_capacity);                /* Configure Batt Mon */
      void ConfigVoltageDivider(float _k ,float _scaling_factor); /* Configure Voltage Divider */
      unsigned int GetId(void);                                /* API for reading the battery id */
      String GetSn(void);                                      /* API for reading the battery serial number */
      unsigned int GetType(void);                              /* API for reading the battery type */
      float GetVoltage(void);                                   /* API for battery voltage read */
      unsigned int GetPercent(void);                           /* API for battery Percent read */
      float GetCapacity(void);                          /* API for battery Capacity read */
      float GetMaxCapacity(void);                          /* API for battery Capacity read */
      void run (void);                                         /* Battery Monitor internal function */
};

#endif /* __BATTERY_MON__*/