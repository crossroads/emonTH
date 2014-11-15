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

#define RF_freq RF12_433MHZ                                                // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 10;                                                  // emonTx RFM12B node ID
const int networkGroup = 210;                                           // emonTx RFM12B wireless network group - needs to be same as emonBase and emonGLCD needs to be same as emonBase and emonGLCD

const int UNO = 1;                                                      // Set to 0 if your not using the UNO bootloader (i.e using Duemilanove) - All Atmega's shipped from OpenEnergyMonitor come with Arduino Uno bootloader
#include <avr/wdt.h>                                                    // the UNO bootloader 

#include <RFu_JeeLib.h>                                                     // Download JeeLib: http://github.com/jcw/jeelib
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
  
typedef struct { int pulse;} PayloadTX;
PayloadTX emontx;                                                        // neat way of packaging data for RF comms

const int LEDpin = 9;                                                   //emonTH LED pin

// Pulse counting settings 
long pulseCount = 0;                                                    // Number of pulses, used to measure energy.
unsigned long lastTime;                                       // Used to measure power.                                               // power and energy                                                         // 1000 pulses/kwh = 1 pulse per wh - Number of pulses per wh - found or set on the meter.


void setup() 
{
  Serial.begin(9600);
  Serial.println("emonTH Pulse Counting example");
  delay(100);
             
  rf12_initialize(nodeID, RF_freq, networkGroup);                          // initialize RF
  rf12_sleep(RF12_SLEEP);

  pinMode(LEDpin, OUTPUT);                                              // Setup indicator LED
  digitalWrite(LEDpin, HIGH);
  
  attachInterrupt(0, onPulse, FALLING);                                 // KWH interrupt attached to IRQ 0  = Digita 2 - hardwired to emonTx V3 terminal block 
  
  if (UNO) 
    wdt_enable(WDTO_8S);  
}

void loop() 
{ 
  emontx.pulse = pulseCount; pulseCount=0; 
  send_rf_data();  // *SEND RF DATA* - see emontx_lib

  Serial.println(emontx.pulse);

  emontx_sleep(10);                                                     // sleep or delay in seconds - see emontx_lib
  digitalWrite(LEDpin, HIGH); delay(2); digitalWrite(LEDpin, LOW);      // flash LED
}


// The interrupt routine - runs each time a falling edge of a pulse is detected
void onPulse()                  
{
  // Firstly, disable further interrupts here (as a kind of switch debouncing).
  detachInterrupt(0);
  unsigned long pulseTime = micros();
  if ((pulseTime - lastTime) > 500000) // 0.5s
  {
    lastTime = pulseTime;        //used to measure time between pulses.
    ++pulseCount;
    Serial.println(pulseCount);
  }
  
  attachInterrupt(0, onPulse, FALLING);
}
