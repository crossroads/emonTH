/*
  emonTH V1.4 Low Power DHT22 Humidity & Temperature & DS18B20 Temperature Node Example 

  Checkes at startup for presence of a DS18B20 temp sensor , DHT22 (temp + humidity) or both
  If it finds both sensors the temperature value will be taken from the DS18B20 (external) and DHT22 (internal) and humidity from DHT22
  If it finds only DS18B20 then no humidity value will be reported
  If it finds only a DHT22 then both temperature and humidity values will be obtained from this sesor
  
  Technical hardware documentation wiki: http://wiki.openenergymonitor.org/index.php?title=EmonTH
 
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson
  Builds upon JCW JeeLabs RF12 library, Arduino and Martin Harizanov's work

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
	- RFu JeeLib		https://github.com/openenergymonitor/RFu_jeelib   //library to work with CISECO RFu328 module
	- DHT22 Sensor Library  https://github.com/adafruit/DHT-sensor-library - be sure to rename the sketch folder to remove the '-'
        - OneWire library	http://www.pjrc.com/teensy/td_libs_OneWire.html
	- DallasTemperature	http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip

  Recommended node ID allocation
  ------------------------------------------------------------------------------------------------------------
  -ID-	-Node Type- 
  0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
  1-4     - Control nodes 
  5-10	- Energy monitoring nodes
  11-14	--Un-assigned --
  15-16	- Base Station & logging nodes
  17-30	- Environmental sensing nodes (temperature humidity etc.)
  31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
  -------------------------------------------------------------------------------------------------------------
*/

/*
  emonTx V3 Pulse example -----------------------------------------

  Many meters have pulse outputs, including electricity meters: single phase, 3-phase, 
  import, export.. Gas meters, Water flow meters etc

  The pulse output may be a flashing LED or a switching relay (usually solid state) or both.

  In the case of an electricity meter a pulse output corresponds to a certain amount of 
  energy passing through the meter (Kwhr/Wh). For single-phase domestic electricity meters
  (eg. Elster A100c) each pulse usually corresponds to 1 Wh (1000 pulses per kwh).  

  The code below detects the falling edge of each pulse and increment pulseCount
  
  It calculated the power by the calculating the time elapsed between pulses.
  
  Read more about pulse counting here:
  http://openenergymonitor.org/emon/buildingblocks/introduction-to-pulse-counting
 
 -----------------------------------------emonTH Hardware Connections-----------------------------
 
 Connect the pulse input into emonTH terminal block port 4 (IRQ 0 / Digital 2)
 See: http://wiki.openenergymonitor.org/index.php?title=EmonTH
 
 
 If your using an optical counter (e.g TSL256) you should connecting the power pin direct to the 3.3V (terminal block 2) or 5V (if running off 5V USB) (terminal port 1) and GND (terminal port 3)
 
 emonTx V3 Terminal block: 
 port 1: 5V
 port 2: 3.3V
 port 3: GND
 port 4: IRQ 0 / Dig2
 
 
 
 
 -----------------------------------------

  -----------------------------------------
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson, Trystan Lea
  Builds upon JeeLabs RF12 library and Arduino

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
 	- RFu JeeLib		https://github.com/openenergymonitor/rfu_jeelib

  Other files in project directory (should appear in the arduino tabs above)
	- emontx_lib.ino
*/

/*Recommended node ID allocation
------------------------------------------------------------------------------------------------------------
-ID-	-Node Type- 
0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
1-4     - Control nodes 
5-10	- Energy monitoring nodes
11-14	--Un-assigned --
15-16	- Base Station & logging nodes
17-30	- Environmental sensing nodes (temperature humidity etc.)
31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
-------------------------------------------------------------------------------------------------------------
*/

// See block comment above for library info
#include <avr/power.h>
#include <avr/sleep.h>
#include <RFu_JeeLib.h>                                                 
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"

const int UNO = 1;                          // Set to 0 if your not using the UNO bootloader (i.e using Duemilanove) - All Atmega's shipped from OpenEnergyMonitor come with Arduino Uno bootloader
#include <avr/wdt.h>                        // the UNO bootloader 
const boolean debug=0;                      //Set to 1 to few debug serial output, turning debug off increases battery life

#define RF_freq RF12_433MHZ                 // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 21;                               // EmonTH temperature RFM12B node ID - should be unique on network
const int networkGroup = 210;               // EmonTH RFM12B wireless network group - needs to be same as emonBase and emonGLCD
                                                                      // DS18B20 resolution 9,10,11 or 12bit corresponding to (0.5, 0.25, 0.125, 0.0625 degrees C LSB), lower resolution means lower power
const int LEDpin = 9;                                                   //emonTH LED pin

// Pulse counting settings 
volatile long pulseCount = 0;               // Number of pulses
volatile unsigned long lastTime = 0;        //                                                                                               // 1000 pulses/kwh = 1 pulse per wh - Number of pulses per wh - found or set on the meter.

const int time_between_readings= 1;                                   // in minutes
const int TEMPERATURE_PRECISION=11;                                   // 9 (93.8ms),10 (187.5ms) ,11 (375ms) or 12 (750ms) bits equal to resplution of 0.5C, 0.25C, 0.125C and 0.0625C
const int ASYNC_DELAY = 375;                                          // 9bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms

ISR(WDT_vect) { Sleepy::watchdogEvent(); }                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

// Hardwired emonTH pin allocations 
const int DS18B20_PWR=5;
const int DHT22_PWR=6;
const int LED=9;
const int BATT_ADC=1;
const int ONE_WIRE_BUS=19;
const int DHTPIN = 18;   

// Humidity code adapted from ladyada' example                        // emonTh DHT22 data pin
// Uncomment whatever type you're using!
// #define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
boolean DHT22_status;                                                 // create flag variable to store presence of DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
boolean DS18B20;                                                      // create flag variable to store presence of DS18B20 

struct Payload 
{                                                      // RFM12B RF payload datastructure
  int temp;
  int temp_external;
  int humidity;    
  int battery; 
  int pulse;  	                                      
};

Payload emonth;

int numSensors; 
//addresses of sensors, MAX 4!!  
byte allAddress [4][8];                                              // 8 bytes per address

//################################################################################################################################
//################################################################################################################################
void setup() 
{ 
  pinMode(LED,OUTPUT); digitalWrite(LED,HIGH);                       // Status LED on
   
  rf12_initialize(nodeID, RF_freq, networkGroup);                       // Initialize RFM12B
  
  // Send RFM12B test sequence (for factory testing)
  for (int i=10; i>-1; i--)                                         
  {
    emonth.temp=i; 
    rf12_sendNow(0, &emonth, sizeof emonth);
    delay(100);
  }
  rf12_sendWait(2);
  emonth.temp = 0;
  emonth.pulse = 0;
  // end of factory test sequence
  
  rf12_sleep(RF12_SLEEP);
  
  // debug = (Serial); //if serial UART to USB is connected show debug O/P. If not then disable serial - DOES NOT WORK http://openenergymonitor.org/emon/node/3930
  
  if (debug)
  {
    Serial.begin(9600);
    Serial.println("emonTH - Firmware V1.2"); 
    Serial.println("OpenEnergyMonitor.org");
    Serial.print("Node: "); 
    Serial.print(nodeID); 
    Serial.print(" Freq: "); 
    if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
    if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
    if (RF_freq == RF12_915MHZ) Serial.print("915Mhz"); 
    Serial.print(" Network: "); 
    Serial.println(networkGroup);
    delay(100);
  }
  
  pinMode(DHT22_PWR,OUTPUT);
  pinMode(DS18B20_PWR,OUTPUT);
  pinMode(BATT_ADC, INPUT);
  digitalWrite(DHT22_PWR,LOW);

  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  ACSR |= (1 << ACD);                     // disable Analog comparator    
  if (!debug) 
    power_usart0_disable();   //disable serial UART
  power_twi_disable();                    //Disable the Two Wire Interface module.
  // power_timer0_disable();              //don't disable necessary for the DS18B20 library
  power_timer1_disable();
  power_spi_disable();
 
  //################################################################################################################################
  // Test for presence of DHT22
  //################################################################################################################################
  digitalWrite(DHT22_PWR,HIGH);
  doDelayInternal(2000);                                                // wait 2s for DH22 to warm up
  dht.begin();
  float h = dht.readHumidity();                                         // Read Humidity
  float t = dht.readTemperature();                                      // Read Temperature
  digitalWrite(DHT22_PWR,LOW);                                          // Power down
  
  if (isnan(t) || isnan(h))                                             // check if returns are valid, if they are NaN (not a number) then something went wrong!
  {
    if (debug==1) Serial.println(" - Unable to find DHT22 Sensor..trying agin"); delay(100);
    Sleepy::loseSomeTime(1500); 
    float h = dht.readHumidity();  float t = dht.readTemperature();
    if (isnan(t) || isnan(h))   
    {
      if (debug==1) Serial.println(" - Unable to find DHT22 Sensor for 2nd time..giving up"); 
      DHT22_status=0;
    } 
  } 
  else 
  {
    DHT22_status=1;
    if (debug==1) Serial.println("Detected DHT22 temp & humidity sensor");  
  }   
 
  //################################################################################################################################
  // Setup and for presence of DS18B20
  //################################################################################################################################
  digitalWrite(DS18B20_PWR, HIGH); delay(50); 
  sensors.begin();
  sensors.setWaitForConversion(false);                             //disable automatic temperature conversion to reduce time spent awake, conversion will be implemented manually in sleeping http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/ 
  numSensors=(sensors.getDeviceCount()); 
  
  byte j=0;                                        // search for one wire devices and
                                                   // copy to device address arrays.
  while ((j < numSensors) && (oneWire.search(allAddress[j])))  j++;
  digitalWrite(DS18B20_PWR, LOW);
  
  if (numSensors==0)
  {
    if (debug) Serial.println("No DS18B20 detected");
    DS18B20=0; 
  } 
  else 
  {
    DS18B20 = true; 
    if (debug) 
    {
      Serial.print("Detected "); Serial.print(numSensors); Serial.println(" DS18B20");
      if (DHT22_status) 
        Serial.println("DS18B20 and DHT22 found, assuming DS18B20 is external sensor");
    }
  }
  
  //################################################################################################################################
  
  // Serial.print(DS18B20); Serial.print(DHT22_status);
  
  // if (debug==1) delay(200);
  
  // Pulse setup start!
  pinMode(2, INPUT_PULLUP); // We use pullup to reduce bouncing.
  attachInterrupt(0, onPulse, FALLING);                                 // KWH interrupt attached to IRQ 0  = Digita 2 - hardwired to emonTx V3 terminal block 
  
  if (debug) delay(200);
   
  digitalWrite(LED,LOW);
} // end of setup

void readDS18B20()
{
  digitalWrite(DS18B20_PWR, HIGH); doDelay(50); 
  for (int j=0;j<numSensors;j++) 
    sensors.setResolution(allAddress[j], TEMPERATURE_PRECISION);      // and set the a to d conversion resolution of each.
  sensors.requestTemperatures();                                        // Send the command to get temperatures
  doDelay(ASYNC_DELAY); //Must wait for conversion, since we use ASYNC mode
  float temp=(sensors.getTempC(allAddress[0]));
  digitalWrite(DS18B20_PWR, LOW);
  if ((temp < 125.0) && (temp > -40.0))
  {
    if (DHT22_status==0) emonth.temp=(temp*10);            // if DHT22 is not present assume DS18B20 is primary sensor (internal)
    if (DHT22_status==1) emonth.temp_external=(temp*10);   // if DHT22 is present assume DS18B20 is external sensor wired into terminal block
  }
}

void readDHT22()
{
  if (debug) Serial.println("readDHT22");
  digitalWrite(DHT22_PWR, HIGH);                                                                                                  // Send the command to get temperatures
  doDelayInternal(2000);                                             //sleep for 1.5 - 2's to allow sensor to warm up
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  emonth.humidity = ((dht.readHumidity())*10);

  float temp=(dht.readTemperature());
  if ((temp<85.0) && (temp>-40.0)) 
    emonth.temp = (temp*10);
  // Check if any reads failed and exit early (to try again).
  if (debug )
  {
    if (isnan(temp))
      Serial.println("Failed to read from DHT sensor!"); 
    else
      Serial.print("Raw temp: "); Serial.println(temp);
  }
  digitalWrite(DHT22_PWR, LOW);
}

void doDebug()
{
  if (debug) 
  {
    if (DS18B20)
    {
      Serial.print("DS18B20 Temperature: ");
      if (DHT22_status) Serial.print(emonth.temp_external/10.0); 
      if (!DHT22_status) Serial.print(emonth.temp/10.0);
      Serial.print("C, ");
    }
    
    if (DHT22_status)
    {
      Serial.print("DHT22 Temperature: ");
      Serial.print(emonth.temp/10.0); 
      Serial.print("C, DHT22 Humidity: ");
      Serial.print(emonth.humidity/10.0);
      Serial.print("%, ");
    }
    
    Serial.print("Battery voltage: ");  
    Serial.print(emonth.battery/10.0);
    Serial.print("V ");
    Serial.print("Pulse count: ");
    Serial.println(emonth.pulse);
    delay(100);
  }
}

void sendRadio()
{
  if (debug)
  {  
    Serial.println("SendRadio");
    delay(20);
  }
  power_spi_enable();  
  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, &emonth, sizeof emonth);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
  power_spi_disable();  
  // digitalWrite(LED,HIGH);
  // dodelay(100);
  // digitalWrite(LED,LOW);
}
//################################################################################################################################
//################################################################################################################################
void loop()
//################################################################################################################################
{ 
  if (!DS18B20 && !DHT22_status)        //if neither DS18B20 or DHT22 is detected flash the LED then goto forever sleep
  {
    for (int i=0; i<20; i++)
    {
      digitalWrite(LED, HIGH); delay(200); digitalWrite(LED,LOW); delay(200);
    }
    cli();                                      //stop responding to interrupts 
    Sleepy::powerDown();                        //sleep forever
  }

  if (DS18B20)
  {
    readDS18B20();
  }
  
  if (DHT22_status)
  { 
    readDHT22();
  }
  
  emonth.battery=int(analogRead(BATT_ADC)*0.03225806);                    //read battery voltage, convert ADC to volts x10
  
  emonth.pulse = pulseCount; pulseCount=0; 
  
  doDebug();
 
  sendRadio();
  
  doDelay(58000); // 58 seconds, as with the 2 second delay for reading makes a minute.
}

void doDelay(unsigned long ms)
{
  // because the timing of loseSomeTime is very approximate, we only want to power down in steps of 1s 
  // to increase timing accuracy.
  const unsigned long SLEEP_TIME = 5000;
  const unsigned long reps = ms / SLEEP_TIME;
  const unsigned long extra = ms % SLEEP_TIME;
  for (int i = 0; i != reps; ++i)
  {
    doDelayInternal(SLEEP_TIME);
  }
  doDelayInternal(extra);
}

void doDelayInternal(unsigned long ms)  
{
  // We need to modify the normal dodelay function here found in the examples
  // because we will be 'interrupted' by the interrupts before the timer
  // has expired, so then we need to power down for the remainder of the expected time again.
  unsigned long millisOutput = millis();
  unsigned long expectedEndMillis = millisOutput + ms;
  if (debug)
  {
    Serial.print(ms);Serial.print(", mo:");Serial.print(millisOutput);Serial.print(", e:");Serial.print(expectedEndMillis);Serial.println(" for dodelay");Serial.flush();
  }
  byte success = 1;
  do 
  {
    // Save Analog to Digital Converter registers - why? I know not!
    byte oldADCSRA=ADCSRA;
    byte oldADCSRB=ADCSRB;
    byte oldADMUX=ADMUX;
    
    success = Sleepy::loseSomeTime(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)    
    ADCSRA=oldADCSRA;         // restore ADC state
    ADCSRB=oldADCSRB;
    ADMUX=oldADMUX;
    if (!success)
    {
      unsigned long current = millis();
      ms = (expectedEndMillis > current) ? expectedEndMillis - current : 0;
      if (debug)
      {
        Serial.print("Not successful: ");Serial.print(current); Serial.print(" ms remaining: "); Serial.println(ms);Serial.flush();
      }
    }
  }
  while (!success && ms >= 16);
  // Minimum power down time is 16 millis.
}

// The interrupt routine - runs each time a falling edge of a pulse is detected
void onPulse()                  
{
  // No need to disable interrupts here as they are already disabled.
  unsigned long pulseTime = micros();
  if ((lastTime > pulseTime) || (pulseTime - lastTime > 500000)) // 0.5s or wrapped!
  {
    lastTime = pulseTime;        //used to measure time between pulses.
    ++pulseCount;
  }
}


