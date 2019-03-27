//---------------------------------------------------REKA INIT-------------------------------------------------

#include <Wire.h>

#define  REKA_FW 0x10
#define  REKA_ID_1 0x100000
#define  REKA_ID_2 0x100000

//---------------------------------------------------BLUETOOTH INIT--------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI reka_control(REKA_CONTROL_SPI_CS, REKA_CONTROL_SPI_IRQ, REKA_CONTROL_SPI_RST); //using 8 7 4
Adafruit_BluefruitLE_SPI reka_beacon(REKA_BEACON_SPI_CS, REKA_BEACON_SPI_IRQ, REKA_BEACON_SPI_RST); //using 10 9 6

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

//----------------------------------------------------GPS INIT---------------------------------------------------

#include <Adafruit_GPS.h>

char gpsdata = "";
uint32_t timer = millis();

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

//----------------------------------------------------VARIABLES---------------------------------------------------

String data_gps = "";
String data_camera = "";
String data_mic = "";
String data_beacon_no = "1";
String data_beacon_id = "";
//uint32_t data_messagetype = "";
uint32_t data_messagetype = 0x20;
String data_1;
String data_2;
String data_3;
int STM32addr = 0x42;

// collection flags 
bool collect_gps = 0;
bool collect_camera = 0;
bool collect_mic = 0;
bool collect_beacon = 0;

void setup() 
{

  Serial.begin(115200); 

  //--------------------------------------------------BLUETOOTH SETUP---------------------------------------------------

  Serial.println(F("Reka HW"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Reka Control: "));

  if ( !reka_control.begin(VERBOSE_MODE) ){
      error(F("Couldn't find Reka Control, make sure it's in Command mode & check wiring?"));
  }
  Serial.println( F("control_OK!") );

  Serial.print(F("Initialising the Reka Beacon: "));

  if ( !reka_beacon.begin(VERBOSE_MODE) ){
      error(F("Couldn't find Reka Beacon, make sure it's in Command mode & check wiring?"));
  }
  Serial.println( F("beacon_OK!") );
  
  if ( FACTORYRESET_ENABLE ) {
      /* Perform a factory reset to make sure everything is in a known state */
      Serial.println(F("Performing a factory reset: "));
      if ( ! reka_control.factoryReset() ) {
        error(F("Couldn't factory reset control"));
      }
      if ( ! reka_beacon.factoryReset() ) {
          error(F("Couldn't factory reset beacon"));
      }
  }

  reka_control.println("AT+GAPDEVNAME=reka_control");
  reka_control.println("ATZ");
  reka_beacon.println("AT+GAPDEVNAME=reka_beacon");
  reka_beacon.println("ATZ");
  
  /* Disable command echo from Bluefruit */
  reka_control.echo(false);
  reka_beacon.echo(false);

  Serial.println("Requesting Reka info:");
  /* Print Bluefruit information */
  reka_control.info();
  reka_beacon.info();
  
  Serial.println(F("Please use Reka app to connect to control"));
  Serial.println();

  reka_control.verbose(false);  // debug info is a little annoying after this point!
  reka_beacon.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! reka_control.isConnected()) {
      delay(500);
  }
  Serial.println("control connected");

  //---------------------------------------------------GPS SETUP-----------------------------------------------
  delay(5000);
  Serial.println("Initializing GPS...");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // turn on recommended minumum data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1Hz update rate 
  delay(1000);
  Serial.println("GPS initialized!");
}

void loop() {

    //-------------------------------COLLECT CAMERA-----------------------------------
    
//    data_camera = "";
//
//    Wire.requestFrom(STM32addr, 15);
//    while (Wire.available()) { // slave may send less than requested
//       char c = Wire.read(); // receive a byte as character
//       data_camera.concat(c);    
//    }
//
//    if (data_camera.length() == 15){
//      collect_camera = 1;
//    }else{
//      //Serial.println("ERROR[-4]: incorrect camera data length \n");
//    }

    //--------------------------------COLLECT ALL DATA-------------------------------------

    String test = "";
    String beacon_name = "";
    char c = GPS.read();
    
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();

    // approximately every 7 seconds or so, print out the current stats
    if (millis() - timer > 7000) { 
      timer = millis(); // reset the timer 

      // DATA 1: BLE
      data_1.concat("$!");
      data_1.concat(String(REKA_FW, HEX));
      data_1.concat(String(REKA_ID_1, HEX));
      data_1.concat(String(REKA_ID_2, HEX));
      data_1.concat("!");
      data_1.concat(String(data_messagetype, HEX));
      data_1.concat("!");

      // DATA 2: GPS
      if (GPS.fix) {
        // try this for sending out right number of characters 
        data_2.concat(GPS.latitude); // need to figure out how to send 6 decimal places? check this with Laura
        Serial.print("Latitude: ");
        Serial.print(GPS.latitude,6); // print latitude with 6 decimal places 
        data_2.concat(',');
        data_2.concat(GPS.lat);
        Serial.print(" ");
        Serial.print(GPS.lat);
        Serial.print("\n");
        data_2.concat(',');
        data_2.concat(GPS.longitude);
        Serial.print("Longitude: ");
        Serial.print(GPS.longitude,6);
        data_2.concat(',');
        Serial.print(" ");
        data_2.concat(GPS.lon);
        Serial.print(GPS.lon);
        Serial.print("\n");
        data_2.concat("!");
        }
      else {
        delay(500);
      }
      data_2.concat("!");

      // DATA 3: CAMERA, MIC, BEACON
      
      if(collect_camera == 1){
        data_3.concat(data_camera);
        collect_camera = 0;
      }
      data_3.concat("!");
      if(collect_mic == 1){
        data_3.concat(data_mic);
        collect_mic = 0;
      }
      data_3.concat("!");

      // COLLECT BEACON
      if (collect_beacon == 0) {
        Serial.println("No beacon connected");
        if(reka_beacon.isConnected()){
          reka_beacon.println("AT+BLEUARTRX");
          reka_beacon.readline();
          test = reka_beacon.buffer;
//          Serial.println(test);
          if(test.equals("bedroom")==1){
            data_beacon_id = "6";
            collect_beacon = 1;
            Serial.println("bedroom connected");
//            beacon_name = "bedroom";
          }
          else if(test.equals("kitchen")==1){
            data_beacon_id = "7";
            collect_beacon = 1;
            Serial.println("kitchen connected");
//            beacon_name = "kitchen";
          }
          else if(test.equals("sister")==1){
            data_beacon_id = "8";
            collect_beacon = 1;
            Serial.println("sister connected");
//            beacon_name = "sister";
          }
        }
        data_3.concat("!");
      }
      else {
        data_3.concat(data_beacon_no);
        data_3.concat("!");
        data_3.concat(data_beacon_id);
        if (!reka_beacon.isConnected()){
          collect_beacon = 0;
        }
      }
      data_3.concat("#");

      reka_control.print("AT+BLEUARTTX=");
      reka_control.println(data_1);
      delay(500);
      reka_control.print("AT+BLEUARTTX=");
      reka_control.println(data_2);
      delay(500);
      reka_control.print("AT+BLEUARTTX=");
      reka_control.println(data_3);
      delay(500);
      data_1 = "";
      data_2 = "";
      data_3 = "";
    }
}
