/*******************************************************************************
 * Copyright (c) 2017 Erik Lemcke
 * Sketch is based upon several other sketches, a lot of code comes from https://github.com/jpmeijers/RN2483-Arduino-Library
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a LoRaWAN packet with GPS coordinates, temperature and humidity
 * over the TTN network. Assumed is you have a SI7021 and a GY-NEO6MV2 coldered on to your loralem
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * 
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * 
 *******************************************************************************/

#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "i2c_SI7021.h"
SI7021 si7021;

static const int GPSRXPin = 6, GPSTXPin = 7;
static const int RNRXPin = 10, RNTXPin = 11, RNResetPin = 12;

static const uint32_t GPSBaud = 9600;
static const uint32_t RNBaud = 9600;

SoftwareSerial GPSSerial(GPSRXPin, GPSTXPin);
TinyGPSPlus gps;
 
SoftwareSerial RNSerial(RNRXPin, RNTXPin); // RX, TX
rn2xx3 myLora(RNSerial);

// the setup routine runs once when you press reset:
void setup() 
{
  RNSerial.listen();
  Serial.begin(115200); //serial port to computer

  initialize_radio();
  delay(2000);

   si7021.initialize();
}

void initialize_radio()
{
  RNSerial.begin(RNBaud); //serial port to radio
  RNSerial.listen();
  Serial.println(F("Startup"));
  
  //reset rn2483
  pinMode(RNResetPin, OUTPUT);
  digitalWrite(RNResetPin, LOW);
  delay(500);
  digitalWrite(RNResetPin, HIGH);

  delay(100); 
  RNSerial.flush();

  myLora.autobaud();

  bool join_result = false;
  
  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  join_result = myLora.initABP("2601112B", "CD915C83B62CA5379240898E418DEF64", "7DBD363CF5D058B7F3A2A18D4A07530E"); 
    
  while(!join_result)
  {
    Serial.println(F("Unable to join. Are your keys correct, and do you have TTN coverage?"));
    delay(1000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println(F("Successfully joined TTN"));
  myLora.sendRawCommand("mac set dr 2"); //set spread factor to 10
  //Serial.println(myLora.sendRawCommand("radio set pwr 14"));
}

void  getGpsCoordinate (long *locationx){
  GPSSerial.begin(GPSBaud);

  long  lat;
  long  lon;

  while (lat == 0){
    delay(100);
    
    while (GPSSerial.available() > 0)
        gps.encode(GPSSerial.read());
  
     if (gps.location.isUpdated())
    {
      lat = (long) (gps.location.lat() * 1000000);
      lon = (long) (gps.location.lng() * 1000000);

     locationx[0] = lat;
     locationx[1] = lon;
    }
  }
}

void loop() 
{
    long location[2];
  
    static float  temp;
    static float  humidity;
    
    si7021.getTemperature(temp);
    si7021.getHumidity(humidity);
    si7021.triggerMeasurement();
        
    int16_t int_temp = (int16_t)(temp * 100);
    int16_t int_humidity = (int16_t)(humidity * 100);
    
    uint8_t coords[12];

    coords[8] = int_temp >> 8;
    coords[9] = int_temp & 0xFF;

    coords[10] = int_humidity >> 8;
    coords[11] = int_humidity & 0xFF;

    getGpsCoordinate (&location[0]);

    Serial.print(F("TXing: "));
    Serial.print(location[0]);
    Serial.print(F(" "));
    Serial.print(location[1]);
    Serial.println(F(" "));

    coords[0] = location[0] >> 24;
    coords[1] = location[0] >> 16;
    coords[2] = location[0] >> 8;
    coords[3] = location[0];

    coords[4] = location[1] >> 24;
    coords[5] = location[1] >> 16;
    coords[6] = location[1] >> 8;
    coords[7] = location[1];

    RNSerial.listen();

    Serial.print(F("TXing: "));
    Serial.println(temp);
    
    Serial.print(F("TXing: "));
    Serial.println(humidity);
    
    myLora.txBytes(coords, sizeof(coords));

    
    
    //wait 5 minutes
    delay(5*60000);
    
}
