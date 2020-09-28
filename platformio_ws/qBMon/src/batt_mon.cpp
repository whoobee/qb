
#include "batt_mon.h"

/* Constructor */
BatteryMonitor::BatteryMonitor(unsigned int _id, String _sn, unsigned int _type, 
            int _adc_in_pin, float _min_voltage, 
            float _max_voltage, float _max_capacity)
{
   ID = _id;
   SN = _sn;
   TYPE = _type;
   inputPin = _adc_in_pin;
   MIN_BATT_VOLTAGE = _min_voltage;
   MAX_BATT_VOLTAGE = _max_voltage;
   MIN_BATT_CAPACITY = 0;
   MAX_BATT_CAPACITY = _max_capacity;
}

/* API for configuration of the voltage divider */
void BatteryMonitor::ConfigVoltageDivider(float _k ,float _scaling_factor)
{
   K = _k;
   SCALING_FACTOR = _scaling_factor;
}

/* Get voltag from ADC input */
float BatteryMonitor::ReadADCVoltage(void)
{
   unsigned int index;
   float voltage;

   /* read from the sensor: */
   readings[readIndex] = analogRead(inputPin);

   /* ROS debug info */
   stringSensorVal = SN + " ADC value: " + String(readings[readIndex], DEC);

   /* add the reading to the total: */
   for(index = 0; index < NUM_OF_READINGS; index++)
   {
      total += readings[readIndex];
   }
   /* calculate the average: */
   average = total / NUM_OF_READINGS;
   total = 0;

   /* calculate ADC voltage */
   voltage = average * K;

   /* ROS debug info */
   stringSensorVolt = SN + " ADC voltage: " + String(voltage, 3);
   
   /* advance to the next position in the array: */
   readIndex++;

   /* if we're at the end of the array... */
   if (readIndex >= NUM_OF_READINGS)
   {
      /* ...wrap around to the beginning: */
      readIndex = 0;
   }
  
   return voltage;
}

/* API for getting the battery id */
unsigned int BatteryMonitor::GetId(void)
{
   return ID;
}

/* API for getting the battery serial number */
String BatteryMonitor::GetSn(void)
{
   return SN;
}

/* API for getting the battery type */
unsigned int BatteryMonitor::GetType(void)
{
   return TYPE;
}

/* API for getting the battery capacity */
float BatteryMonitor::GetVoltage(void)
{
   return currentBatteryVoltage;
}

/* API for getting the battery capacity */
unsigned int BatteryMonitor::GetPercent(void)
{
   return currentBatteryPercent;
}

/* API for getting the battery capacity */
float BatteryMonitor::GetCapacity(void)
{
   return currentBatteryCapacity;
}

/* API for getting the battery capacity */
float BatteryMonitor::GetMaxCapacity(void)
{
   return MAX_BATT_CAPACITY;
}

/* Cyclic function for battery info calculation */
void BatteryMonitor::run(void)
{
   /*****************************/
   /* Calculate battery voltage */
   /*****************************/
   /* Get battery voltage cosidering the volrate divider */
   currentBatteryVoltage = ReadADCVoltage() * SCALING_FACTOR;
   stringBattVolt = SN + " voltage: " + String(currentBatteryVoltage, 3);

   /*****************************/
   /* Calculate battery percent */
   /*****************************/
   if (currentBatteryVoltage > MAX_BATT_VOLTAGE) /* Over charged - set 100% */
   {
      currentBatteryPercent = 100;
   }
   else if (currentBatteryVoltage < MIN_BATT_VOLTAGE) /* Totaly drained - set 0% */
   {
      currentBatteryPercent = 0;
   }
   else /* calculate percent */
   {
      /* Calculate usable voltage */
      currentBatteryPercent = (currentBatteryVoltage - MIN_BATT_VOLTAGE) * (100 / (MAX_BATT_VOLTAGE - MIN_BATT_VOLTAGE));
   }
   stringBattPer = SN + " percent: " + String(currentBatteryPercent, DEC);

   /******************************/
   /* Calculate battery capacity */
   /******************************/
   currentBatteryCapacity = (MAX_BATT_CAPACITY * currentBatteryPercent)/100u;
   /* Calculate battery capacity */
   if (currentBatteryCapacity > MAX_BATT_CAPACITY) /* Over charged - set 100% */
   {
      currentBatteryCapacity = MAX_BATT_CAPACITY;
   }
   else if (currentBatteryCapacity < MIN_BATT_CAPACITY) /* Totaly drained - set 0% */
   {
      currentBatteryCapacity = MIN_BATT_CAPACITY;
   }
   /* Debug data */
   stringBattCapacity = SN + " capacity: " + String(currentBatteryCapacity, 3);

   Serial.println(stringSensorVal);
   Serial.println(stringSensorVolt);
   Serial.println(stringBattVolt);
   Serial.println(stringBattPer);
   Serial.println(stringBattCapacity);
}