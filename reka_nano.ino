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





//-------------BLUETOOTH INIT---------------

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


//--------------GPS INIT-------------------a

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "BluefruitConfig.h"
char gpsdata = "";
String gpsdata_line="";

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





void setup() {
  // put your setup code here, to run once:

  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);


//-------------BLUETOOTH SETUP---------------


  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !reka_control.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit1, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK1!") );

  if ( !reka_beacon.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit2, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK2!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! reka_control.factoryReset() ){
      error(F("Couldn't factory reset1"));
    }
    if ( ! reka_beacon.factoryReset() ){
      error(F("Couldn't factory reset2"));
    }
  }


  reka_control.println("AT+GAPDEVNAME=reka_control");
  reka_control.println("ATZ");
  reka_beacon.println("AT+GAPDEVNAME=reka_beacon");
  reka_beacon.println("ATZ");

  /* Disable command echo from Bluefruit */
  reka_control.echo(false);
  reka_beacon.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  reka_control.info();
  reka_beacon.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  reka_control.verbose(false);  // debug info is a little annoying after this point!
  reka_beacon.verbose(false);  // debug info is a little annoying after this point!


  /* Wait for connection */
  while (! reka_control.isConnected() || ! reka_beacon.isConnected()) {
      delay(500);
  }


  // LED Activity command is only supported from 0.6.6
  if ( reka_control.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED1 activity to " MODE_LED_BEHAVIOUR));
    reka_control.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }

  if ( reka_beacon.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED2 activity to " MODE_LED_BEHAVIOUR));
    reka_beacon.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }



//-------------GPS SETUP---------------

  mySerial.begin(9600);
  delay(2000);
  Serial.println("Get version!");
  mySerial.println(PMTK_Q_RELEASE);
  
  // you can send various commands to get it started
  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
//  mySerial.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);

  mySerial.println(PMTK_SET_NMEA_UPDATE_1HZ);



  


  

}

void loop() {
  // put your main code here, to run repeatedly:






//-------------BLUETOOTH LOOP---------------

sendUserInput();
sendGPSDataToControl();






  

//haiToBLE(reka_control);
//haiToBLE(reka_beacon);


delay(1000); 

checkDataFromBLE(reka_control);
checkDataFromBLE(reka_beacon);
  

}


bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}



//-------------BLUETOOTH FCNS---------------

void sendUserInput(void){
  // Check for user input
  char inputs[BUFSIZE+1];

  if ( getUserInput(inputs, BUFSIZE) )
  {
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

void haiToBLE(Adafruit_BluefruitLE_SPI channel){

  channel.print("AT+BLEUARTTX=");
  channel.println("hai");

  return;

}

void checkDataFromBLE(Adafruit_BluefruitLE_SPI channel){

  channel.println("AT+BLEUARTRX");
  channel.readline();
  if (strcmp(channel.buffer, "OK") != 0){
      Serial.print(F("[Recv1] ")); 
      Serial.println(channel.buffer);
      channel.waitForOK();
  }

  return;

}  




void sendGPSDataToControl(){

    if(mySerial.available() != 0){
      
      gpsdata_line = gpsdata_line + gpsdata;
      Serial.print(gpsdata);

      gpsdata = mySerial.read();
      if(gpsdata == '$'){
        //gpsdata_line = gpsdata_line + gpsdata;
        gpsdata_line.concat(gpsdata);
        Serial.print(gpsdata);
        gpsdata = mySerial.read();
      }else if (gpsdata == -1){
        gpsdata = mySerial.read();
      }else{
          while(gpsdata!='$'){
            if(gpsdata!= -1){
              //gpsdata_line = gpsdata_line + gpsdata;
              gpsdata_line.concat(gpsdata);
              Serial.print(gpsdata);
            }
            gpsdata = mySerial.read();
          }
      }

      if(gpsdata_line != ""){
          //Serial.println(gpsdata_line);
          reka_control.print("AT+BLEUARTTX=");
          reka_control.println(gpsdata_line);
          gpsdata_line = "";
      }

  }

  return;

}
