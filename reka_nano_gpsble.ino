
//---------------------------------------------------REKA INIT-------------------------------------------------

#include <Wire.h>

#define  REKA_FW 0x10
#define  REKA_ID_1 0x100000
#define  REKA_ID_2 0x100000

//---------------------------------------------------BLUETOOTH INIT--------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI reka_control(REKA_CONTROL_SPI_CS, REKA_CONTROL_SPI_IRQ, REKA_CONTROL_SPI_RST); //using 8 7 4

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

//----------------------------------------------------GPS INIT---------------------------------------------------

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

char gpsdata = "";
uint32_t timer = millis();

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

String gps_get(void){    
    Serial.println("gps_get");
    
}

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
  
    if ( FACTORYRESET_ENABLE ) {
        /* Perform a factory reset to make sure everything is in a known state */
        Serial.println(F("Performing a factory reset: "));
        if ( ! reka_control.factoryReset() ) {
          error(F("Couldn't factory reset control"));
        }
    }
  
    reka_control.println("AT+GAPDEVNAME=reka_control");
    reka_control.println("ATZ");
  
    /* Disable command echo from Bluefruit */
    reka_control.echo(false);
  
    Serial.println("Requesting Reka info:");
    /* Print Bluefruit information */
    reka_control.info();
  
    Serial.println(F("Please use Reka app to connect to control"));
    Serial.println();
  
    reka_control.verbose(false);  // debug info is a little annoying after this point!
  
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
}

void loop() {

    String data_gps = "";
    String data_camera = "";
    String data_mic = "";
    String data_beacon_no = "";
    String data_beacon_id = "";
    uint32_t data_messagetype = "";
    String data_1;
    String data_2;
    String data_3;
    int STM32addr = 0x42;

    // collection flags 
    bool collect_gps = 0;
    bool collect_camera = 0;
    bool collect_mic = 0;
    bool collect_beacon = 0;
  
    //--------------------------------COLLECT GPS-------------------------------------
   
    char c = GPS.read();

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();
  
    // approximately every 5 seconds or so, print out the current stats
    if (millis() - timer > 5000) { 
      timer = millis(); // reset the timer 
      data_1.concat("$!");
      data_1.concat(String(REKA_FW, HEX));
      data_1.concat(String(REKA_ID_1, HEX));
      data_1.concat(String(REKA_ID_2, HEX));
      data_1.concat("!");
      data_1.concat(String(data_messagetype, HEX));
      data_1.concat("!");
      data_3.concat("!");
      if(collect_mic == 1){
          data_3.concat(data_mic);
          collect_mic = 0;
      }
      data_3.concat("!");
      if(collect_beacon == 1){
         data_3.concat(data_beacon_no);
         data_3.concat("!");
         data_3.concat(data_beacon_id);
         collect_beacon = 0;
      }else{
          data_3.concat("!");
      }
      data_3.concat("#");
      if (GPS.fix) {
        data_2.concat(GPS.latitude); // need to figure out how to send 4 decimal places
        Serial.println(GPS.latitude,4); // print latitude with 4 decimal places 
        data_2.concat(',');
        data_2.concat(GPS.lat);
        data_2.concat(',');
        data_2.concat(GPS.longitude);
        data_2.concat(',');
        data_2.concat(GPS.lon);
        data_2.concat("!");
      }
      else {
        delay(500);
      }
      data_2.concat("!");
      reka_control.print("AT+BLEUARTTX=");
      reka_control.println(data_1);
      delay(500);
      reka_control.print("AT+BLEUARTTX=");
      reka_control.println(data_2);
      delay(500);
      reka_control.print("AT+BLEUARTTX=");
      reka_control.println(data_3);
      data_1 = "";
      data_2 = "";
      data_3 = "";
    }
}
