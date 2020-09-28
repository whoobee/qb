
#include "debug.h"

static byte ledState = 0;

void Debug_Init(void)
{
   Serial.begin(115200);
   pinMode(LED_BUILTIN, OUTPUT);
}

void taskDebug(int id_)
{
   if (ledState)
   {
      digitalWrite(LED_BUILTIN, LOW);
      ledState = 0;
   }
   else
   {
      digitalWrite(LED_BUILTIN, HIGH);
      ledState = 1;
   }
   Serial.print("LED state: ");
   Serial.println(ledState);
}