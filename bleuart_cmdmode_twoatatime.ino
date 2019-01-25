/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


bool flag_1 = 0;
bool flag_2 = 0;
/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);







//----test----
Adafruit_BluefruitLE_SPI ble_1(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST); //using 8 7 4
Adafruit_BluefruitLE_SPI ble_2(10, 9, 6); //using 10 9 6






/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble_1.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit1, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK1!") );

  if ( !ble_2.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit2, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK2!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble_1.factoryReset() ){
      error(F("Couldn't factory reset1"));
    }
    if ( ! ble_2.factoryReset() ){
      error(F("Couldn't factory reset2"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble_1.echo(false);
  ble_2.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble_1.info();
  ble_2.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble_1.verbose(false);  // debug info is a little annoying after this point!
  ble_2.verbose(false);  // debug info is a little annoying after this point!


  /* Wait for connection */
  while (! ble_1.isConnected() || ! ble_2.isConnected()) {
      delay(500);
  }

//  while (! ble_2.isConnected()) {
//      delay(500);
//  }

  // LED Activity command is only supported from 0.6.6
  if ( ble_1.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED1 activity to " MODE_LED_BEHAVIOUR));
    ble_1.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }

  if ( ble_2.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED2 activity to " MODE_LED_BEHAVIOUR));
    ble_2.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check for user input
  char inputs[BUFSIZE+1];

  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble_1.print("AT+BLEUARTTX=");
    ble_1.println(inputs);

    
    ble_2.print("AT+BLEUARTTX=");
    ble_2.println(inputs);

    // check response stastus
    if (! ble_1.waitForOK() ) {
      Serial.println(F("Failed to send1?"));
    }

    if (! ble_2.waitForOK() ) {
      Serial.println(F("Failed to send2?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble_1.println("AT+BLEUARTRX");
  ble_2.println("AT+BLEUARTRX");
  ble_1.readline();
  ble_2.readline();
  if (strcmp(ble_1.buffer, "OK") != 0){
      Serial.print(F("[Recv1] ")); 
      Serial.println(ble_1.buffer);
      ble_1.waitForOK();
      flag_1 = 1;
  }
  if ((strcmp(ble_2.buffer, "OK") != 0)){
      Serial.print(F("[Recv2] ")); 
      Serial.println(ble_2.buffer);
      ble_2.waitForOK();
      flag_2 = 1;
  }
  if((flag_1 && flag_2) == 0){
    //nodata
    return;
  }

  flag_1 = 0;
  flag_2 = 0;



//
//  // Check for incoming characters from Bluefruit
//  ble_1.println("AT+BLEUARTRX");
//  ble_1.readline();
//  if (strcmp(ble_1.buffer, "OK") == 0) {
//    // no data
//    return;
//  }
//
//  // Some data was found, its in the buffer
//  Serial.print(F("[Recv1] ")); Serial.println(ble_1.buffer);
//  ble_1.waitForOK();
//
//  ble_2.println("AT+BLEUARTRX");
//  ble_2.readline();
//  if (strcmp(ble_2.buffer, "OK") == 0) {
//    // no data
//    return;
//  }
//  
//  // Some data was found, its in the buffer
//  Serial.print(F("[Recv2] ")); Serial.println(ble_2.buffer);
//  ble_2.waitForOK();


}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
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
