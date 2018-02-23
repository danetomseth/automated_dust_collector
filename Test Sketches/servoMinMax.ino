#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long serialdata;
int inbyte;

long data;
int servoNum = 0;


const int NUMBER_OF_GATES = 7;
const int NUMBER_OF_TOOLS = 5;

//String tools[NUMBER_OF_TOOLS] = {"Table Saw","Planer","Jointer","Miter Saw","Band Saw"};
String tools[NUMBER_OF_TOOLS] = {"Miter Saw","Table Saw","Band Saw"};

String gateLabels[NUMBER_OF_GATES] = {"Jointer", "Planer", "Miter", "South Y", "North Y", "Band Saw", "Table Saw"};

//int voltSensor[NUMBER_OF_TOOLS] = {A0,A1,A2,A3,A4};
int voltSensor[NUMBER_OF_TOOLS] = {A0,A1,A2};
long int voltBaseline[NUMBER_OF_TOOLS] = {0,0,0,0,0};
int currentServo = 9;
int goToPos = 150;

int gateMinMax[NUMBER_OF_GATES][2] = {
  /*open, close*/
  {420, 340}, // 4 - Jointer
  {170, 230}, // 3 - Planer
  {390, 300}, // 5 - Miter Saw
  {180, 250}, // 0 - South Y
  {130, 200}, // 1 - North Y
  {140, 210}, // 6 - Band Saw
  {120, 205}, // 2 - Table saw
  // {SERVOMIN, SERVOMAX} // Floor Sweep
};

int servoChannel[NUMBER_OF_GATES] = {9,10,11,12,13,14,15};

int gates[NUMBER_OF_TOOLS][NUMBER_OF_GATES] = {
  {0,0,1,0,0,0,0}, // table
  {1,0,0,1,0,0,0}, // planer
  {1,0,0,0,1,0,0}, // jointer
  {1,0,0,0,0,1,0}, // miter
  {0,1,0,0,0,0,1}, // bandsaw
};

void setup()
{
  Serial.begin(9600);
  Serial.println("STARTING");
  pwm.begin();
  pwm.setPWMFreq(50);
  
  
//  
 
 

  delay(10);
}

void loop()
{
  Serial.println("--------------");
//  Serial.println("1) Test Gate");
//  Serial.println("2) Test Tool");
//  Serial.println("3) Close All");
//  Serial.println("4) Open All");
  Serial.println("Gate?");
  currentServo = getSerial();
  if(currentServo == 100) {
    minMaxTest();
  }
  else {


    Serial.println("Position?");
    goToPos = getSerial();
    
    while(goToPos > 100) {
      setPosition(currentServo, goToPos);
      Serial.println("Position?");
      goToPos = getSerial();
    }
  }
//  switch (data) {
//    case 1:
//      // statements
//      Serial.println("--------------");
//      Serial.println("Which gate?");
//      data = getSerial();
//      Serial.print("Testing Gate: ");
//      Serial.println(data);
//      delay(1000);
//      openGate(data);
//      delay(1000);
//      closeGate(data);
//
//      break;
//    case 2:
//      Serial.println("--------------");
//      for(int s=0;s<NUMBER_OF_TOOLS;s++){
//        Serial.print(s);
//        Serial.print(") ");
//        Serial.println(tools[s]);
//      }
//      Serial.println("--------------");
//      data = getSerial();
//      Serial.print("Testing: ");
//      Serial.println(tools[data]);
//      for(int s=0;s<NUMBER_OF_GATES;s++){
//        int pos = gates[data][s];
//        if(pos == 1){
//          openGate(s);    
//        } else {
//          closeGate(s);
//        }
//      }
//      break;
//    case 3:
//      Serial.println("--------------");
//      Serial.println("CLOSE ALL");
//      for(int s=0;s<NUMBER_OF_GATES;s++){
//        closeGate(s);
//        delay(1000);
//      }
//      break;
//    case 4:
//    Serial.println("--------------");
//    Serial.println("OPEN ALL");
//      for(int s=0;s<NUMBER_OF_GATES;s++){
//        openGate(s);
//        delay(1000);
//      }
//      break;
//    default:
//      Serial.println("Default");
//  }

  inbyte = 0;
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


void minMaxTest() {
  for(int i = 0; i<NUMBER_OF_GATES; i++) {
    Serial.println(gateLabels[i]);
    openGate(i);
    delay(1000);
    closeGate(i);
    delay(1000);
    openGate(i);
    delay(1000);
    closeGate(i);
    delay(1000);
  }
}

void setPosition(uint8_t num, int pos) {
  Serial.print("SERVO:");
  Serial.println(num);
  Serial.print("POS:");
  Serial.println(pos);
  pwm.setPWM(num, 0, pos);
//  int moveMin = pos - 50;
//  int moveMax = pos + 50;
//  for(int i = 0; i < 2; i++) {
//    pwm.setPWM(num, 0, moveMin);
//    delay(1000);
//    pwm.setPWM(num, 0, moveMax);
//    delay(1000);  
//  }
  
}

void closeGate(uint8_t num){
  pwm.setPWM(servoChannel[num], 0, gateMinMax[num][0]);
}
void openGate(uint8_t num){
  pwm.setPWM(servoChannel[num], 0, gateMinMax[num][1]);
}

void closeAll(){
  for(int s=0;s<NUMBER_OF_GATES;s++){
        closeGate(s);
        delay(500);
      }
}

void neutralAll(){
  for(int s=9;s<NUMBER_OF_GATES;s++){
        pwm.setPWM(s, 0, 200);
        delay(1000);
      }
}

