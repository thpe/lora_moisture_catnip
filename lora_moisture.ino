// TinyLoRa DHT22 - ABP TTN Packet Sender (Multi-Channel)
// Tutorial Link: https://learn.adafruit.com/the-things-network-for-feather/using-a-feather-32u4
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Copyright 2015, 2016 Ideetron B.V.
//
// Modified by Brent Rubell for Adafruit Industries, 2018
// Copyright 2020 Thomas Petig
/************************** Configuration ***********************************/
#include <TinyLoRa.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <I2CSoilMoistureSensor.h>
#include <Wire.h>
#include "ttnconfig.h"

#define LED_WAN 18
#define LED_SENS 17
#define LED_BAT 16
#define VBATPIN A0
#define VTEST   A1

/************************** Example Begins Here ***********************************/
// Data Packet to Send to TTN
unsigned char loraData[9];

// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 30;

// Pinout for Adafruit Feather 32u4 LoRa
//TinyLoRa lora = TinyLoRa(7, 8, 4);

// Pinout for Adafruit Feather M0 LoRa
TinyLoRa lora = TinyLoRa(3, 8, 4);

I2CSoilMoistureSensor sensor1(0x21);

I2CSoilMoistureSensor sensor0(0x20);

TinyGPS gps;
static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

void setup()
{
  delay(2000);
  Serial.begin(9600);

  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_WAN, OUTPUT);
  pinMode(LED_SENS, OUTPUT);
  pinMode(LED_BAT, OUTPUT);
  digitalWrite (LED_WAN, HIGH);
  // Initialize LoRa
  Serial.print("Starting LoRa...");
  // define multi-channel sending
  lora.setChannel(MULTI);
  // set datarate
  lora.setDatarate(SF7BW125);
  if(!lora.begin())
  {
    Serial.println("Failed");
    Serial.println("Check your radio");
    while(true);
  }
  digitalWrite(LED_WAN, LOW);
  digitalWrite(LED_SENS, HIGH);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  Serial.println("OK");
// delay(2000);
  Wire.begin();
//  Wire.setClockStretchLimit(2500);
//  delay(2000);
  Serial.println("sensor...");
  sensor0.begin(true); // reset sensor
  delay(1000); // give some time to boot up
  Serial.print("I2C Soil Moisture Sensor Address: ");
  Serial.println(sensor0.getAddress(),HEX);
  Serial.print("Sensor Firmware version: ");
  Serial.println(sensor0.getVersion(),HEX);
  Serial.println();
  sensor1.begin(true); // reset sensor
  delay(1000); // give some time to boot up
  Serial.print("I2C Soil Moisture Sensor Address: ");
  Serial.println(sensor1.getAddress(),HEX);
  Serial.print("Sensor Firmware version: ");
  Serial.println(sensor1.getVersion(),HEX);
  Serial.println();
  digitalWrite(LED_SENS, LOW);
}

void loop()
{



  uint16_t* cap = (uint16_t*) (loraData + 1);
  int16_t* temp = (int16_t*)  (loraData + 5);

  while (sensor0.isBusy()) delay(50); // available since FW 2.3
  cap[0] = sensor0.getCapacitance();
  temp[0] = sensor0.getTemperature();
  sensor0.sleep(); // available since FW 2.3



  while (sensor1.isBusy()) delay(50); // available since FW 2.3
  cap[1] = sensor1.getCapacitance();
  temp[1] = sensor1.getTemperature();
  sensor1.sleep(); // available since FW 2.3
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  Serial.print("Soil Moisture Capacitance: ");
  Serial.print(cap[0]); //read capacitance register
  Serial.print(", ");
  Serial.print(cap[1]);
  Serial.print(", Temperature: ");
  Serial.print(temp[0]/10.0f);
  Serial.print(", ");
  Serial.println(temp[1]/10.0f);
  if (true) {


  // encode int as bytes
  //byte payload[2];
#if 0
  loraData[0] = measuredvbat - 430;
  loraData[1] = ((uint8_t*)&cap[0])[0];
  loraData[2] = ((uint8_t*)&cap[0])[1];
  loraData[3] = ((uint8_t*)&cap[1])[0];
  loraData[4] = ((uint8_t*)&cap[1])[1];
  loraData[5] = ((uint8_t*)&temp[0])[0];
  loraData[6] = ((uint8_t*)&temp[0])[1];
  loraData[7] = ((uint8_t*)&temp[1])[0];
  loraData[8] = ((uint8_t*)&temp[1])[1];
#endif
  Serial.println("Sending LoRa Data...");
  lora.sendData(loraData, sizeof(loraData), lora.frameCounter);
  Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
  lora.frameCounter++;
  }
  // blink LED to indicate packet sent
  digitalWrite(LED_WAN, HIGH);
  delay(1000);
  digitalWrite(LED_WAN, LOW);

  Serial.println("delaying...");
  delay(sendInterval * 1000);
}
