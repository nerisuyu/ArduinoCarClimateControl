#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <ESP8266WiFi.h>
#include "index.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SSD1306.h>  

void connectToWiFi(const char* ssid, const char* passphrase) {
 Serial.begin(9600);
  // Disconnecting WiFi if it"s already connected.
 WiFi.disconnect();
  // Setting it to Station mode which basically scans for nearby WiFi routers.
 WiFi.mode(WIFI_STA);
  // Begin connecting to WiFi
 WiFi.begin(ssid, passphrase);
 Serial.printf("\nDevice is connecting to WiFi using SSID %s and Passphrase %s.\n", ssid, passphrase);
}

void printAddress(DeviceAddress deviceAddress)
{ 
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}

void displayText(Adafruit_SSD1306 *display,const int x, const int y, const int textsize,const char* text)
{
  display->setTextSize(textsize);
  display->setCursor(x, y);
  display->println(text);
}

void displayText(Adafruit_SSD1306 *display,const int x, const int y, const int textsize,int value)
{
  display->setTextSize(textsize);
  display->setCursor(x, y);
  display->print(value);
}

void displayTemperature(Adafruit_SSD1306 *display,const int x, const int y,const int textsize, const int temperature)
{
  if(temperature==-127)
  {displayText(display,x,y,textsize,"n/c");}
  else
  {displayText(display,x,y,textsize,temperature);};
}

static inline int8_t sign(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

#endif /* FUNCTIONS_H */