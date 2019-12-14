/* ***************************************************************************************************
// Example sketch from John's DIY Playground
// to demonstrate the Moteino IoT device using an RFM69 transceiver attached to it.
// Visit http://www.lowpowerlab.com/ to purchase a Moteino of your own
//
// This sketch will demonstrate a few basic features of the Moteino:
//   1. It will flash the onboard LED located at pin D9
//   2. It will report sensor light levels from a photoresistor on pin A5 to the Home Automation gateway
//   3. It will allow us to control an LED connected to pin D3 remotely
//
// Demonstration of this code can be found on my YouTube channel, called John's DIY Playground
// The channel's URL is http://www.youtube.com/c/johnsdiyplayground
// Software code examples can be found on my GitHub page at
// https://github.com/johnsdiyplayground
// 
// Hardware Required:
// 1. Moteino or Moteino USB with RFM69 transceiver (http://www.lowpowerlab.com/)
// 2. LED
// 3. 2 quantity of 220 ohm resistor for the external LED and photoresistor circuits
//     Put the resistors in series; the LED to ground and the other from photoresistor A5 to ground
// 4. Photoresistor
// 5. USB to micro USB cable for powering the Moteino
// 6. Another USB Moteino with the Pi Gateway code loaded on it and connected to a Raspberry Pi
//      NOTE: this second Moteino must have the same frequency RFM69 transceiver attached to it
// 7. FTDI USB to Moteino programming interface (Not needed if you are using Moteino USB devices)
******************************************************************************************************
*/
#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: http://github.com/lowpowerlab/spiflash
//#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <RFM69_OTA.h>

#define SERIAL_EN          //comment out if you don't want any serial output

//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/TRANSCEIVER SETTINGS/REQUIREMENTS
//*****************************************************************************************************************************
#define GATEWAYID   1    // this is the node ID of your gateway (which is probably tied to a Raspberry Pi)
#define NODEID      50   // must be unique to each sensor node
#define NETWORKID   200  // every node must match the same network ID to hear messages from each other and take actions directly
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino's transceiver! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW      //uncomment only for RFM69HW! Leave out if you have RFM69W!
//*****************************************************************************************************************************

#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif

// Define parameters for the door sensor
#include "QuickMedianLib.h"

int anPin = A2;
int anPin2 = A3;
long anVolt, cm, inches;
long anVolt2, cm2, inches2;
int currentOccupancy = 0, maxOccupancy = 0, totalOccupancy = 0;
int minDist = 80, maxDist = 500;
int values30[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Define our pins
#define onboardLED     9   // Moteino onboard LED is on digital pin 9

              // For example, on Moteino you CANNOT use D2, D8, D10, D11, D12, D13!!

// Now we will define how often to flash our onboard LED 
const long blinkLEDtime = 1000;  // 1000 is milliseconds so this means blink on/off in 1 second intervals
// and we need a way to record the last time we turned the LED on or off
unsigned long lastBlinkTime;

// Now we will define how often to check the door 
const long checkDoorTime = 500;  // 1000 is milliseconds so this means blink on/off in 1 second intervals
// and we need a way to record the last time we turned the LED on or off
unsigned long lastDoorTime;

// The next variable defines how often to check the occupancy value should be sent
const long occupancyCheckTime = 30000;  // 10000 milliseconds is 10 seconds
// and we also record the time we report the photoresistor status with this
unsigned long lastOccupancyReport;

// We don't need to define how often to check commands to turn on/off the external LED.  That's because
// our Moteino will respond to commands to request it come on or off from the Raspberry Pi / Moteino gateway device.
// The only thing we do track is the external LED's current status as on (1) or off (0)
int ledStatus;  

char payload[] = "123 ABCDEFGHIJKLMNOPQRSTUVWXYZ";   //this is for transmitting to the Pi Gateway
byte STATUS;
RFM69 radio;
SPIFlash flash(8, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)

void setup() {
  #ifdef SERIAL_EN
    Serial.begin(SERIAL_BAUD);
  #endif

  // Tell Moteino if our pins are inputs or outputs
  pinMode(onboardLED, OUTPUT);
  pinMode(anPin, INPUT);
  pinMode(anPin2, INPUT);

  // Initialize our onboard and external LEDs to start up turned off (set to low)
  digitalWrite(onboardLED, LOW);
  
  // Initialize our timers for the onboard LED and photoresistor using millis which is the Photon's internal clock
  lastBlinkTime = millis();
  lastOccupancyReport = millis();
  
  char buff[20];  // create a variable called buff (short for "buffer") that can hold up to 20 characters
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  #ifdef IS_RFM69HW
     radio.setHighPower(); //uncomment only for RFM69HW!
  #endif
  radio.encrypt(ENCRYPTKEY);
  
  if (flash.initialize()){
    DEBUGln("EEPROM flash chip found, and is OK ...");
  }
  else 
    DEBUGln("EEPROM flash chip failed to initialize! (is chip present on the Moteino?)");
  
  
  sprintf(payload, "Moteino Example : %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);  // sprintf command creates a string to put in the buff variable
  DEBUGln(payload);  // check what we are transmitting to the gateway receiver
  byte buffLen=strlen(payload);
  radio.sendWithRetry(GATEWAYID, payload, buffLen);
  // We need to send the next line to the gateway to let it know we have an external LED with condition "off" to start with
  radio.sendWithRetry(GATEWAYID, "3300:0", 6);
  delay(5000); 
}

void loop() {
   // wireless programming token check
   // DO NOT REMOVE, or this sensor node will not be wirelessly programmable any more!
   CheckForWirelessHEX(radio, flash, true);

  /* Eric Commented it out this part since we do not receive anything
  // check for messages from the home automation gateway or other Moteino nodes  
      
      if (radio.receiveDone()){
      DEBUG("Msg received from sender ID ");DEBUG('[');DEBUG(radio.SENDERID);DEBUG("] ");
      for (byte i = 0; i < radio.DATALEN; i++)
          DEBUG((char)radio.DATA[i]);
      DEBUGln();
      //first send any ACK to request
      DEBUG("   [RX_RSSI:");DEBUG(radio.RSSI);DEBUGln("]");
      DEBUG("   Payload length = ");
      DEBUGln(radio.DATALEN);  
    
      if (radio.DATALEN==3){
          //check for a web page request from Pi Gateway to turn on or off the external LED
          //we will receive from the Pi Gateway either a message of "LOF" or LON"
          if (radio.DATA[0]=='L' && radio.DATA[1]=='O' && radio.DATA[2]=='F'){    // "LOF" is short for "LED off", keep radio messages short for best results
            digitalWrite(externalLED, LOW); // Tell our external LED to turn OFF
            ledStatus = 0;          // this sets our LED status variable 
            delay(50);
            transmitStatus(9900, ledStatus);   // now we transmit LED status back to the Pi Gateway that we turned the LED off using transmitStatus subroutine
                              // We are passing a unique 4-digit number to identify a unique data type for our Pi Gateway, since it listens
                              // to a lot of sensor nodes in our house.  Each sensor node gets its own set of 4-digit codes.
                              // Any time we transmit an LED status, we will always identify it with sensor code "9900"
          }
          else if (radio.DATA[0]=='L' && radio.DATA[1]=='O' && radio.DATA[2]=='N'){  // "LON" is short for "LED on"
            digitalWrite(externalLED, HIGH);  // Tell our external LED to turn ON
            ledStatus = 1;          // this sets our LED status variable 
            delay(50);
            transmitStatus(9900, ledStatus);   // now we transmit LED status back to the Pi Gateway that we turned the LED off
          }
      }
    } 
    */

    if (radio.ACKRequested())
    {
      radio.sendACK();
      DEBUGln("ACK sent.");
    }

  // check if the onboard LED should be turned on or off
  if ((millis() - lastBlinkTime) >= blinkLEDtime) {
      lastBlinkTime = millis();  // update current time for the next time through the loop
      digitalWrite(onboardLED, !digitalRead(onboardLED));  // this says in one simple line to flip-flop the status of the pin
  }

    // collect door sensor data
  if ((millis() - lastDoorTime) >= checkDoorTime) {
      lastDoorTime = millis();  // update current time for the next time through the loop
      roomCheck();
  }

  // check if it is time to report the photoresistor's value to the Moteino USB gateway (receives data for the Raspberry Pi) 
  if ((millis() - lastOccupancyReport) >= occupancyCheckTime) {
      lastOccupancyReport = millis();   // update current time for the next time through the loop
      Serial.print("transmitting...");
      Serial.println(currentOccupancy);
      
      // publish the actual value of the light level.  We will call this sensor ID data code "9912"
      transmitStatus(3300, currentOccupancy);
  }
}  // this is the end of the loop


// The function below will transmit the LED status to our Moteino USB / Pi Gateway
void transmitStatus(int item, int status){  
    sprintf(payload, "%d:%d", item, status);
    byte buffLen=strlen(payload);
    radio.sendWithRetry(GATEWAYID, payload, buffLen);
    DEBUG("Transmitted payload: ");
    DEBUGln(payload);
    delay(10);
}




void roomCheck () {
  cm = read_sensor(anPin);
//  print_range(1, cm);
  

  cm2 = read_sensor(anPin2);
//  print_range(2, cm2);

  if (cm < minDist || cm > maxDist) {
//    Serial.println("First condition met!");
    // when R detects but L stays put
    if (cm2 > minDist && cm2 < maxDist) {
      Serial.println();
      Serial.println("Welcome!");
      currentOccupancy++;
      totalOccupancy++;
      if (maxOccupancy < currentOccupancy) {
        maxOccupancy = currentOccupancy;
      }
      delay(1300);
    }
  }

  if (cm2 < minDist || cm2 > maxDist) {
    // when L detects but R stays put
    if (cm > minDist && cm < maxDist) {
      Serial.println();
      Serial.println("Goodbye!");
      if (!(currentOccupancy < 1)) {
        currentOccupancy--;
      }
      delay(1300);
    }
  }

  Serial.println();
  Serial.print(currentOccupancy);
  Serial.println(" people are in the room.");
  Serial.print(maxOccupancy);
  Serial.println(" maximum are in the room today.");
  Serial.print(totalOccupancy);
  Serial.println(" people have come to the room.");
}

long read_sensor (int pin){

  for (int i = 0; i < 30; i++) {
    anVolt = analogRead(pin) / 2;
    values30[i] = anVolt * 2.54;
    delay(5);
  }

  long cm = QuickMedian<int>::GetMedian(values30, 30);
  return cm;
}

void print_range (int sensor, long cm){
  if (sensor == 1) {
    Serial.print("   S1");
  }
  else {
    Serial.print("   S2");
  }
  Serial.print("=");
  Serial.print(cm);
  Serial.print(" cm");
//  Serial.print(" ");
//  Serial.println(inches);
}
