/*
 * This code is for the project at 
 * http://www.iliketomakestuff.com/how-to-automate-a-dust-collection-system-arduino
 * All of the components are list on the url above.
 * 
This script was created by Bob Clagett for I Like To Make Stuff
For more projects, check out iliketomakestuff.com
Includes Modified version of "Measuring AC Current Using ACS712"
http://henrysbench.capnfatz.com/henrys-bench/arduino-current-measurements/acs712-arduino-ac-current-tutorial/
Parts of this sketch were taken from the keypad and servo sample sketches that comes with the keypad and servo libraries.
Uses https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include "IRLibAll.h"
#include <IRLibSendBase.h>    // First include the send base
//Now include only the protocols you wish to actually use.
//The lowest numbered protocol should be first but remainder 
//can be any order.
#include <IRLib_P01_NEC.h>    
#include <IRLibCombo.h>     // After all protocols, include this
// All of the above automatically creates a universal sending
// class called "IRsend" containing only the protocols you want.
// Now declare an instance of that sender.

//*** IR Globals ***//
IRsend irSender;

//future implementation of IR recieve to extend remote range
//IRrecvPCI irReceiver(3); 
//IRdecode irDecoder;   



//*** Blast Gate/Servo Globals ***// 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // default address 0x40
const int NUMBER_OF_GATES = 7;
const int NUMBER_OF_TOOLS = 7;
int servoNum = 0;
int currentGate = 9;

String tools[NUMBER_OF_TOOLS] = {"Jointer", "Planer", "Miter", "Band Saw", "Table Saw"}; 
String gateLabels[NUMBER_OF_GATES] = {"Jointer", "Planer", "Miter", "Band Saw", "Table Saw", "South Y", "North Y"};
int gateChannel[NUMBER_OF_GATES] = {9,10,11,14,15,12,13}; // {jointer, planer, miter, bandsaw, tablesaw, south y, north y}
//DC right, Y, miter, bandsaw, saw Y, tablesaw, floor sweep

// throw limits for each gate (open/closed)
int gateMinMax[NUMBER_OF_GATES][2] = {
  // {SERVOMIN, SERVOMAX}
  {420, 340}, // 4 - Jointer
  {170, 230}, // 3 - Planer
  {390, 310}, // 5 - Miter Saw
  {140, 210}, // 6 - Band Saw
  {120, 205}, // 2 - Table saw
  {180, 250}, // 0 - South Y
  {130, 200}, // 1 - North Y
  
};

int gates[NUMBER_OF_TOOLS][NUMBER_OF_GATES] = {
  // {jointer, planer, miter, bandsaw, tablesaw, south y, north y}
  {1,0,0,0,0,1,0}, // jointer
  {0,1,0,0,0,1,0}, // planer
  {0,0,1,0,0,1,0}, // miter
  {0,0,0,1,0,0,1}, // bandsaw
  {0,0,0,0,1,0,0}, // table
  {0,0,0,0,0,1,0}, // SOUTH
  {0,0,0,0,0,0,1}, // NORTH
};

//*** AC Globals ***//
const int NUMBER_OF_SENSORS = 5;
int voltSensor[NUMBER_OF_SENSORS] = {A0,A3,A1,A2,A4}; // jointer(A0), planer(A3), miter(A1), bandsaw(A2), tablesaw(A4)
int mVperAmp = 66; // use 100 for 20A Module and 66 for 30A Module
double ampThreshold = 3;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
int activeTool = 50;// 50 is default for no tool active

//*** Manual Button Globals ***//
const int manualSwitchPin = 13; //for button activated gate
int reading;           // the current reading from the manual input pin
bool buttonPressed = false;
//button debouncing
long time = 0;         // the last time the output pin was toggled
long debounce = 1000;   // the debounce time, increase if the output flickers




//*** DC Globals ***//
int DC_spindown = 2000;
int DC_spinup = 1000; //used if amp draw is too much on breaker

bool collectorIsOn = false;



//*** Program Control ***//
bool DEBUG = true;
bool serialActive = true;
bool initiated = false; //run at startup to close gates and allow AC readings to stabilize
bool serialControl = false;

//Serial.read globals
unsigned long serialdata;
int inbyte;
long data;

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.


void setup(){ 
  Serial.begin(9600);
  while (!Serial) {
   printString("-"); // wait for serial port to connect. Needed for native USB
  }
  pwm.begin();
  pwm.setPWMFreq(50);  // Default is 1000mS
  printLine("-");
  printLine("Program Ready");
  
}

void loop(){
  // use later for button debouncing
  if(!initiated) {
    closeAll();
    delay(1000);
    if(serialActive) {
      printLine("Program Initiated");
    }
    initiated = true;
  }

  if(serialControl) {
    // {"Jointer", "Planer", "Miter", "Band Saw", "Table Saw"};
    printLine(" 0) Jointer , 1) Planer , 2) Miter , 3) Band Saw , 4) Table Saw, 5) SOUTH ON, 6) NORTH ON, 10) EXIT");
    activeTool = getSerial();
    if(activeTool > NUMBER_OF_GATES) {
      serialControl = false;
    }
    else {
      activateTool(); 
      printLine("Waiting to turn off");
      getSerial();
      turnOffDustCollection();
    }

  }
  else {


    reading = digitalRead(manualSwitchPin);


    if (reading == HIGH  && millis() - time > debounce) {
      if (buttonPressed){
        buttonPressed = false;
        irSender.send(NEC,0x10EFE01F,32);
       
      } else{
        buttonPressed = true;
        irSender.send(NEC,0x10EFC03F,32);
      }
      time = millis();
    }
    
     //loop through tools and check
     activeTool = 50;// a number that will never happen
     for(int i=0;i<NUMBER_OF_SENSORS;i++){
        if(checkForAmperageChange(i)){
          activeTool = i;
          exit;
        }
     }
    if(activeTool != 50){
      // use activeTool for gate processing
      if(collectorIsOn == false){
        //manage all gate positions
        activateTool();
      }
    } else{
      if(collectorIsOn == true){
        printLine("Turning DC Off");
        delay(DC_spindown);
        turnOffDustCollection();  
      }
    }
}
}

void activateTool() {
  for(int s=0;s<NUMBER_OF_GATES;s++){
        int pos = gates[activeTool][s];
        if(pos == 1){
          openGate(s);    
        } else {
          closeGate(s);
        }
      }
      // delay(DC_spinup); //not currently used
      turnOnDustCollection();
}


void turnOnDustCollection(){
  if(DEBUG) {
    printLine("DC ON");
  }
  irSender.send(NEC,0x10EFC03F,32);
  collectorIsOn = true;
}
void turnOffDustCollection(){
  if(DEBUG) {
    printLine("DC OFF");
  }
  irSender.send(NEC,0x10EFE01F,32);
  collectorIsOn = false;
  closeAll();
}


bool checkForAmperageChange(int which) {
  Voltage = getVPP(voltSensor[which]);
  VRMS = (Voltage/2.0) *0.707; 
  AmpsRMS = (VRMS * 1000)/mVperAmp;
  if(DEBUG) {
    printString(tools[which]+": ");
    printNum(AmpsRMS);
    printLine(" Amps RMS");
  } 
  
  if(AmpsRMS>ampThreshold) {
    if(!DEBUG) {
      printString(tools[which]+": ");
      printNum(AmpsRMS);
      printLine(" Amps RMS");
    }
    
    return true;
  }
  else {
    return false; 
  }
}

float getVPP(int sensor) {
  float result;
  
  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 1024;          // store min value here
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 250) //sample for 250 mSec
   {
       readValue = analogRead(sensor);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the maximum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;
      
   return result;
 }



void closeGate(uint8_t num){
  if(serialActive) {
    printString("Gate: ");
    printString(gateLabels[num]);
    printString(" --- Pos: ");
    printNumLine(gateMinMax[num][0]);
  }
  pwm.setPWM(gateChannel[num], 0, gateMinMax[num][0]);
}
void openGate(uint8_t num){
  if(serialActive) {
    printString("Gate: ");
    printString(gateLabels[num]);
    printString(" --- Pos: ");
    printNumLine(gateMinMax[num][1]);
  }
  pwm.setPWM(gateChannel[num], 0, gateMinMax[num][1]);
}

void closeAll() {
  for(int i=0; i<NUMBER_OF_GATES; i++) {
    pwm.setPWM(gateChannel[i], 0, gateMinMax[i][0]);
    delay(500);
  }
}

void openAll() {
  for(int i=0; i<NUMBER_OF_GATES; i++) {
    pwm.setPWM(gateChannel[i], 0, gateMinMax[i][1]);
    delay(500);
  }
}


long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read();  
    if (inbyte > 0 && inbyte != '/')
    { 
      serialdata = serialdata * 10 + inbyte - '0';
//      Serial.println(serialdata);
    }
  }
  inbyte = 0;
  return serialdata;
  
}


void printString(String message) {
  if(serialActive) {
    Serial.print(message);
  }
}


void printLine(String message) {
  if(serialActive) {
    Serial.println(message);
  }
}

void printNum(double num) {
  if(serialActive) {
    Serial.print(num);
  }
}

void printNumLine(double num) {
  if(serialActive) {
    Serial.println(num);
  }
}


