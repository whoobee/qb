#pragma once

#include "Debug.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ros.h>

class WiFiHW {
  public:
  WiFiHW(const char* ssid, const char* password, boolean otaSupport);
  void OTAHandling(void);
  boolean IsConnected(void);

  private:
  boolean _otaSupport;
};

WiFiHW::WiFiHW(const char* ssid, const char* password, boolean otaSupport)
{
  _otaSupport = otaSupport;
  
  /* Print status */
  if(DEBUG)
  {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }

  /* Connect the ESP8266 the the wifi AP */
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    /* print pending status */
    if(DEBUG)
    {
      Serial.print(".");
    }
  }
  /* Pint connection status */
  if(DEBUG)
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  if(_otaSupport == true)
  {
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);
  
    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");
  
    // No authentication by default
    // ArduinoOTA.setPassword((const char *)"123");
    ArduinoOTA.onStart([]() {
      if(DEBUG)
      {
        Serial.println("Start");
      }
    });
    ArduinoOTA.onEnd([]() {
      if(DEBUG)
      {
        Serial.println("\nEnd");
      }
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      if(DEBUG)
      {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      }
    });
    ArduinoOTA.onError([](ota_error_t error) {
      if(DEBUG)
      {
        Serial.printf("Error[%u]: ", error);
      }
      if (error == OTA_AUTH_ERROR) if(DEBUG){Serial.println("Auth Failed");}
      else if (error == OTA_BEGIN_ERROR) if(DEBUG){Serial.println("Begin Failed");}
      else if (error == OTA_CONNECT_ERROR) if(DEBUG){Serial.println("Connect Failed");}
      else if (error == OTA_RECEIVE_ERROR) if(DEBUG){Serial.println("Receive Failed");}
      else if (error == OTA_END_ERROR) if(DEBUG){Serial.println("End Failed");}
    });
    ArduinoOTA.begin();
  }
}


void WiFiHW::OTAHandling(void)
{
  if(_otaSupport == true)
  {
    ArduinoOTA.handle();
  }
}
