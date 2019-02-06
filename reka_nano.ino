// do current measurements
//total 28-29mA --> OK
// figure out power scheme
//arduino nano 5V logic, so from 5V rail
// migrate over bluetooth x 2
//make fcns
// migrate over uart
//make fcns


// integrate camera i2c
//make fcns

//TO DO:
//verify beacon read ok
//beacon automatically connect
//packet ids
//can't read beacon
//hex for beacon
//act if camera not running

//TO DO (Hannah):
//not using gps data when not enough characters - less than 65 characters (DONE)
//make sure newline not getting passed through your fcn - remove last two characters of GPS data being sent (DONE)
//testing gps_fix after getting fix from GPS (does this mean just getting an actual gps fix and testing the code?)
//change from + to .concat (DONE)
//need to test functionality of all the above fixes 
//
//script may freeze if camera not sending data (nice to have)

//---------------------------------------------------REKA INIT-------------------------------------------------

#include <Wire.h>

#define  REKA_FW 0x10
#define  REKA_ID_1 0x100000
#define  REKA_ID_2 0x100000

//bool info_message = 0;



//-----2^8-------------------------------------------BLUETOOTH INIT--------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

//ble_1 = reka_control
//ble_2 = reka_beacon
Adafruit_BluefruitLE_SPI reka_control(REKA_CONTROL_SPI_CS, REKA_CONTROL_SPI_IRQ, REKA_CONTROL_SPI_RST); //using 8 7 4
Adafruit_BluefruitLE_SPI reka_beacon(REKA_BEACON_SPI_CS, REKA_BEACON_SPI_IRQ, REKA_BEACON_SPI_RST); //using 10 9 6

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


//----------------------------------------------------GPS INIT---------------------------------------------------

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "BluefruitConfig.h"
char gpsdata = "";

SoftwareSerial mySerial(3, 2);

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_Q_RELEASE "$PMTK605*31"




bool getUserInput(char buffer[], uint8_t maxSize)
{
    // timeout in 100 milliseconds
    TimeoutTimer timeout(100);
  
    memset(buffer, 0, maxSize);
    while ( (!Serial.available()) && !timeout.expired() ) {
        delay(1);
    }
  
    if (timeout.expired()) return false;
  
    delay(2);
    uint8_t count = 0;
    do {
        count += Serial.readBytes(buffer + count, maxSize);
        delay(2);
    } while ( (count < maxSize) && (Serial.available()) );
  
    return true;
}

void sendUserInput(void) {
    // Check for user input
    char inputs[BUFSIZE + 1];
    
    if ( getUserInput(inputs, BUFSIZE) ) {
        // Send characters to Bluefruit
        Serial.print("[Send] ");
        Serial.println(inputs);
      
        reka_control.print("AT+BLEUARTTX=");
        reka_control.println(inputs);
      
      
        reka_beacon.print("AT+BLEUARTTX=");
        reka_beacon.println(inputs);
      
        // check response stastus
        if (! reka_control.waitForOK() ) {
            Serial.println(F("Failed to send1?"));
        }
      
        if (! reka_beacon.waitForOK() ) {
            Serial.println(F("Failed to send2?"));
        }
    }
    
    return;

}

void haiToBLE(Adafruit_BluefruitLE_SPI channel) {
  
    channel.print("AT+BLEUARTTX=");
    channel.println("hai");
  
    return;

}

String beacon_read(Adafruit_BluefruitLE_SPI channel) {

//    channel.println("AT+BLEUARTFIFO=RX");
//    channel.readline();
//    Serial.println(channel.buffer);
//    if (strcmp(channel.buffer, "0")!=0){
//      channel.readline();
//      channel.println("AT+BLEUARTRX");
//      channel.readline();
//      if (strcmp(channel.buffer, "OK") != 0) {
//        //Serial.print(F("[Recv1] "));
//        //Serial.println(channel.buffer);
//        channel.waitForOK();
//        return channel.buffer;
//      }
//    }  
    return ("-1");

}


String gps_get(void){

    String gpsdata_line = "";
    
    if(mySerial.available()==0){
        return ("-1");
    }else{
        if(gpsdata=='$'){
            gpsdata_line.concat(gpsdata);
            gpsdata = mySerial.read();
      
            while (gpsdata != '$') {
                if (gpsdata != -1) {
                    //gpsdata_line = gpsdata_line + gpsdata;
                    gpsdata_line.concat(gpsdata);
                    //Serial.print(gpsdata);
                }
                gpsdata = mySerial.read();
            }
            return gpsdata_line;
        }else{
            while (gpsdata != '$') {
                if (gpsdata == -1) {
                    return ("-1");
                }
                gpsdata = mySerial.read();
                Serial.println(gpsdata);
            }
            return ("-2");
        }
    }

}


String gps_fix(String gps) {
  if (gps.substring(18,19) == "A") {
  //check for gps fix 
    if (gps.length() > 65) {
    // parse the gps data if the string length seems acceptable 
        
      String lat_coords = gps.substring(20,29);
      String north_south = gps.substring(30,31);
      String long_coords = gps.substring(32,42);
      String east_west = gps.substring(43,44);
      
      lat_coords.remove(4,1); // remove decimal places
      long_coords.remove(5,1); // remove decimal places 
      String lat_degrees = lat_coords.substring(0,2);
      String lat_decimaldegs = String((lat_coords.substring(2,8) + "00").toInt()/60);
      String long_degrees = long_coords.substring(0,3);
      String long_decimaldegs = String((long_coords.substring(3,9) + "00").toInt()/60);
      
      String lat_sign = "";
      if (north_south == "S") {
          lat_sign = "-";
      }
      
      String long_sign = "";
      if (east_west == "W") {
          long_sign = "-";
      }

      lat_sign.concat(lat_degrees);
      lat_sign.concat(".");
      lat_sign.concat(lat_decimaldegs);
      lat_sign.concat(",");
      lat_sign.concat(long_sign);
      lat_sign.concat(long_degrees);
      lat_sign.concat(".");
      lat_sign.concat(long_decimaldegs);

      String send_gps = lat_sign;

      int send_gps_length = send_gps.length();
      // remove the new line character at the end of gps_data (assume "\n" is two characters)
      send_gps.remove(send_gps_length-2,2);
  
      return (send_gps);
    } else{
      // error code for string not being the right length 
      return ("-2");
      }
  }
  else {
      // error code for no fix 
      return("-1");
  }
}


void setup() {


   
    // put your setup code here, to run once:
  
    //while (!Serial);  // required for Flora & Micro
    delay(500);
  
    Serial.begin(115200);

    Wire.begin();        // join i2c bus (address optional for master)  
  
    //--------------------------------------------------BLUETOOTH SETUP---------------------------------------------------
  
  
    Serial.println(F("Reka HW"));
    Serial.println(F("---------------------------------------"));
  
    /* Initialise the module */
    Serial.print(F("Initialising the Reka Control and Beacon: "));
  
    if ( !reka_control.begin(VERBOSE_MODE) ){
        error(F("Couldn't find Reka Control, make sure it's in CoMmanD mode & check wiring?"));
    }
    Serial.println( F("control_OK!") );
  
    if ( !reka_beacon.begin(VERBOSE_MODE) ){
        error(F("Couldn't find Reka Beacon, make sure it's in CoMmanD mode & check wiring?"));
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
  
  
    //    /* Wait for connection */
    //    while (! reka_control.isConnected() || ! reka_beacon.isConnected()) {
    //        delay(500);
    //    }
  
  
    /* Wait for connection */
    while (! reka_control.isConnected()) {
        delay(500);
    }
    Serial.println("control connected");

  
    //info_message = 1;
  
    // LED Activity command is only supported from 0.6.6
    if ( reka_control.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
        // Change Mode LED Activity
        Serial.println(F("******************************"));
        Serial.println(F("Change LED1 activity to " MODE_LED_BEHAVIOUR));
        reka_control.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
        Serial.println(F("******************************"));
    }

    if ( reka_beacon.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
        // Change Mode LED Activity
        Serial.println(F("******************************"));
        Serial.println(F("Change LED2 activity to " MODE_LED_BEHAVIOUR));
        reka_beacon.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
        Serial.println(F("******************************"));
    }

  
  
  
    //---------------------------------------------------GPS SETUP-----------------------------------------------
  
    mySerial.begin(9600);
    delay(2000);
    Serial.println("Get GPS version!");
    mySerial.println(PMTK_Q_RELEASE);
  
    // you can send various commands to get it started
    mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    //  mySerial.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  
    mySerial.println(PMTK_SET_NMEA_UPDATE_1HZ);






    //-----------------------------------------------NON GLOBAL VAR-----------------------------------------------


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




  
  // put your main code here, to run repeatedly:

    bool collect_gps = 0;
    bool collect_camera = 0;
    bool collect_mic = 0;
    bool collect_beacon = 0;
  
  
    //--------------------------------COLLECT GPS-------------------------------------
    
    data_gps = "";
    data_gps = gps_get();
    //Serial.println(data_gps);
    if (data_gps == "-1"){
        Serial.print("ERROR[-1]: no GPS data available \n");
    }else if (data_gps == "-2"){
        Serial.print("ERROR[-2]: incomplete GPS parse, collect next data set \n");
    }else{
        //Serial.print(data_gps);
        //data_gps = gps_fix(data_gps);
        //if(data_gps == "-1"){
        //    Serial.print("ERROR[-3]: no GPS fix \n");
        //}else if(data_gps == "-2") {
        //    Serial.print("ERROR[-4]: data invalid, cannot be parsed \n");
        //}else {
        if(data_gps.length()>=40){
            collect_gps = 1;
        }else{
            collect_gps = 0;
        }
        //}
    }
    //-------------------------------COLLECT CAMERA-----------------------------------
    
    data_camera = "";

    Wire.requestFrom(STM32addr, 15);
    while (Wire.available()) { // slave may send less than requested
       char c = Wire.read(); // receive a byte as character
       data_camera.concat(c);    
    }

    if (data_camera.length() == 15){
      collect_camera = 1;
    }else{
      //Serial.println("ERROR[-4]: incorrect camera data length \n");
    }

    //-------------------------------COLLECT MIC--------------------------------------
    
    data_mic = "";
    //collect_mic = 1;
    
    //-------------------------------COLLECT BEACONS----------------------------------
    
    data_beacon_id = "";
    data_beacon_no = "";

//
//    if(!reka_beacon.isConnected()) {
//        data_beacon_id = beacon_read(reka_beacon);
//        if (data_beacon_id == "-1"){
//            Serial.print("ERROR[-5]: failed to read beacon data \n");
//        }else{
//            collect_beacon = 1;
//            data_beacon_no = "1";
//        }
//    }
//

    if(reka_beacon.isConnected()){
      //Serial.println("success");
      //delay(10000);
      //data_beacon_id = beacon_read(reka_beacon);
      //if (data_beacon_id == "-1"){
      //    Serial.print("ERROR[-5]: failed to read beacon data \n");
      //}else{
      //    collect_beacon = 1;
      //    data_beacon_no = "1";
      //    Serial.print("beacon sending");
      //}
      data_beacon_id = "10000000";
      data_beacon_no = "10";  
      collect_beacon = 1;    
    }else{
      //Serial.println("no beacon pair");
    }
      
    //-------------------------------SEND PACKET--------------------------------------
    
    //if(info_message == 1){
    //    data_messagetype = 0x10;      //info message
    //    info_message = 0;
    //}else{
        data_messagetype = 0x20;      //data message
    //}
    
    
    
    
    //data_1 = "$!" + String(REKA_FW, HEX) + String(REKA_ID_1, HEX) + String(REKA_ID_2, HEX) + "!" + String(data_messagetype, HEX) + "!";

    
    data_1.concat("$!");
    data_1.concat(String(REKA_FW, HEX));
    data_1.concat(String(REKA_ID_1, HEX));
    data_1.concat(String(REKA_ID_2, HEX));
    data_1.concat("!");
    data_1.concat(String(data_messagetype, HEX));
    data_1.concat("!");
    
    if(collect_gps == 1){
       data_2.concat(data_gps);
       collect_gps = 0;
    }
    data_2.concat("!");

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
    if(collect_beacon == 1){
       data_3.concat(data_beacon_no);
       data_3.concat("!");
       data_3.concat(data_beacon_id);
       collect_beacon = 0;
    }else{
        data_3.concat("!");
    }
    data_3.concat("#");


    Serial.println(data_1);
    Serial.println(data_2);
    Serial.println(data_3);



    
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

    //delay(6000);

}
