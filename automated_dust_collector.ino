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

//IR Globals
//IRrecvPCI myReceiver(3); //create receiver and pass pin number
//IRdecode myDecoder;   //create decoder

IRsend irSender;
 
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

// our servo # counter

#define SERVOMIN  180 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  425 // this is the 'maximum' pulse length count (out of 4096)

const int OPEN_ALL = 100;
const int CLOSE_ALL = 99;

boolean buttonTriggered = 0;
boolean powerDetected = 0;
boolean collectorIsOn = 0;
boolean DEBUG = true;
int DC_spindown = 2000;
int DC_spinup = 1000;

 const int NUMBER_OF_TOOLS = 7;
const int NUMBER_OF_GATES = 7;

unsigned long serialdata;
int inbyte;

long data;
int servoNum = 0;


String tools[NUMBER_OF_TOOLS] = {"Table Saw","Planer","Jointer","Miter Saw","Band Saw"}; // "Floor Sweep"
int voltSensor[NUMBER_OF_TOOLS] = {A4,A3,A0,A1,A2};


long int voltBaseline[NUMBER_OF_TOOLS] = {0,0,0,0,0};




String gateLabels[NUMBER_OF_GATES] = {"Jointer", "Planer", "Miter", "South Y", "North Y", "Band Saw", "Table Saw"};




//DC right, Y, miter, bandsaw, saw Y, tablesaw, floor sweep
//Set the throw of each gate separately, if needed

// {gate 0, gate 1, gate 2, gate 3, gate 4, gate 5, gate 6, gate 7}
int gateMinMax[NUMBER_OF_GATES][2] = {
  /*open, close*/
  {420, 340}, // 4 - Jointer
  {170, 230}, // 3 - Planer
  {390, 310}, // 5 - Miter Saw
  {180, 250}, // 0 - South Y
  {130, 200}, // 1 - North Y
  {140, 210}, // 6 - Band Saw
  {120, 205}, // 2 - Table saw
  // {SERVOMIN, SERVOMAX} // Floor Sweep
};

int servoChannel[NUMBER_OF_GATES] = {9,10,11,12,13,14,15}; // {jointer, planer, miter, south y, north y, bandsaw, tablesaw}

int gates[NUMBER_OF_TOOLS][NUMBER_OF_GATES] = {
  // {jointer, planer, miter, south y, north y, bandsaw, tablesaw}

  {0,0,0,0,0,0,1}, // table
  {0,1,0,1,0,0,0}, // planer
  {1,0,0,1,0,0,0}, // jointer
  {0,0,1,1,0,0,0}, // miter
  {0,0,0,0,1,1,0}, // bandsaw
  {0,0,0,1,0,0,0}, // SOUTH
  {0,0,0,0,1,0,0}, // NORTH
};
//int gates[NUMBER_OF_TOOLS][NUMBER_OF_GATES] = {
//  {1,0,0,0,0,1,0}, // miter
//  {0,0,1,0,0,0,0}, // table
//  {0,1,0,0,0,0,1}, // bandsaw
//};

const int manualSwitchPin = 13; //for button activated gate, currently NOT implemented
int currentGate = 9;

int mVperAmp = 66; // use 100 for 20A Module and 66 for 30A Module
double ampThreshold = 3;
int activeTool = 50;// a number that will never happen


double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

//button debouncing
int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
int buttonPressed = false;

bool initiated = false;
bool testRun = false;

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 1000;   // the debounce time, increase if the output flickers

void setup(){ 
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Default is 1000mS
  
 //record baseline sensor settings
 //currently unused, but could be used for voltage comparison if need be.
//  for(int i=0;i<NUMBER_OF_TOOLS;i++){
//    pinMode(voltSensor[i],INPUT);
//    voltBaseline[i] = analogRead(voltSensor[i]);
//    if(DEBUG) {
//      Serial.print(tools[i]);
//      Serial.print(" baseline: ");
//      Serial.println(voltBaseline[i]);
//    } 
//  }
  
  
}

void loop(){
  // use later for button debouncing
  if(!initiated) {
    for(int i = 0; i < NUMBER_OF_GATES; i++) {
          currentGate = servoChannel[i];
          closeGate(currentGate, i);
    }
    initiated = true;
  }

  if(testRun) {
    Serial.println(" 0) Table Saw , 1) Planer , 2) Jointer , 3) Miter Saw , 4) Band Saw, 5) SOUTH ON, 6) NORTH ON");
    activeTool = getSerial();
    if(activeTool > NUMBER_OF_TOOLS) {
      testRun = false;
    }
    else {
      activateTool();
      Serial.println("Waiting to turn off");
      getSerial();
      turnOffDustCollection();
    }

  }
  else {


  reading = digitalRead(manualSwitchPin);


  if (reading == HIGH  && millis() - time > debounce) {
    if (buttonPressed){
      Serial.println("off");
      buttonPressed = false;
//      turnOffDustCollection();
      
      irSender.send(NEC,0x10EFE01F,32);
     
    } else{
      Serial.println("on");
      buttonPressed = true;
      irSender.send(NEC,0x10EFC03F,32);
//      turnOnDustCollection();
       
    }
    time = millis();
  }
  
//  Serial.println("----------");
   //loop through tools and check
   activeTool = 50;// a number that will never happen
   for(int i=0;i<NUMBER_OF_TOOLS;i++){
      if( checkForAmperageChange(i)){
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
      Serial.println("Turning DC Off");
      delay(DC_spindown);
      turnOffDustCollection();  
    }
  }
}
}
boolean checkForAmperageChange(int which) {
  Voltage = getVPP(voltSensor[which]);
  VRMS = (Voltage/2.0) *0.707; 
  AmpsRMS = (VRMS * 1000)/mVperAmp;
 
  Serial.print(tools[which]+": ");
    Serial.print(AmpsRMS);
    Serial.println(" Amps RMS");
  if(AmpsRMS>ampThreshold) {
    if(DEBUG) {
    
  }
    return true;
  }
  else {
    return false; 
  }
}
void turnOnDustCollection(){
  if(DEBUG) {
    Serial.println("DC ON");
  }
  irSender.send(NEC,0x10EFC03F,32);
  collectorIsOn = true;
}
void turnOffDustCollection(){
  if(DEBUG) {
    Serial.println("DC OFF");
  }
  irSender.send(NEC,0x10EFE01F,32);
  collectorIsOn = false;
//  closeAll();
}
 
float getVPP(int sensor)
{
  float result;
  
  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 1024;          // store min value here
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 250) //sample for 1 Sec
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

void activateTool() {
  for(int s=0;s<NUMBER_OF_GATES;s++){
        int pos = gates[activeTool][s];
        if(pos == 1){
          currentGate = servoChannel[s];
          Serial.print("Open Gate: ");
          Serial.println(currentGate);
          openGate(currentGate, s);    
        } else {
          currentGate = servoChannel[s];
          closeGate(currentGate, s);
        }
      }
      delay(DC_spinup);
      turnOnDustCollection();
}

void closeGate(uint8_t num, uint8_t tool){
  Serial.print("Gate: ");
  Serial.print(num);
  Serial.print(" --- Pos: ");
  Serial.println(gateMinMax[tool][0]);
  pwm.setPWM(num, 0, gateMinMax[tool][0]);
}
void openGate(uint8_t num, uint8_t tool){
  Serial.print("Gate: ");
  Serial.print(num);
  Serial.print(" --- Pos: ");
  Serial.println(gateMinMax[tool][1]);
  pwm.setPWM(num, 0, gateMinMax[tool][1]);
}

void closeAll() {
  for(int i=0; i<NUMBER_OF_GATES; i++) {
    pwm.setPWM(servoChannel[i], 0, gateMinMax[i][0]);
    delay(500);
  }
}

void openAll() {
  for(int i=0; i<NUMBER_OF_GATES; i++) {
    pwm.setPWM(servoChannel[i], 0, gateMinMax[i][1]);
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

