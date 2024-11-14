#include "thingProperties.h"
#include <QTRSensors.h>  //install library

#include <L298N.h>
QTRSensors qtr; //makes it easier to type funcs 

#define EN_A 2
#define IN1_A 3
#define IN2_A 4

#define EN_B 5
#define IN1_B 6
#define IN2_B 7

float RMcorrection = 1.0;
float LMcorrection = 0.95;
float lspeed;
float lspeedfinal;
float rspeedfinal;
float Kd;
float Ki;
float Kp;

const uint8_t SensorCount = 5;  //declare number of sensors in the format the library likes
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

uint16_t position;
float previousError;

//declare variables 
float PIDvalue;  

float multiP = 0.0;  //scaling vals
float multiI = 0.0;
float multiD = 0.0;

L298N motorR(EN_A, IN1_A, IN2_A); //initiate motor objects
L298N motorL(EN_B, IN1_B, IN2_B);


void setup() {
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A1, A2, A3, A4, A5 }, SensorCount);
  pinMode(D12, OUTPUT);
  pinMode(D13, OUTPUT);
  
  if ((ArduinoCloud.connected() == 0))  {
    digitalWrite(D13, HIGH);
  }else {
      //calibration start
  digitalWrite(D12, HIGH);          //turn on led, connected to digital pin 12, to indicate we are calibrating
  for (uint16_t i = 0; i < 5; i++)  //400 repeats, so a few minutes worth of calibration
  {
    qtr.calibrate();  //must ensure that sensors are placed at height which will be used for the line follow sequence
    delay(250);
  }
  }
  digitalWrite(D12, LOW);
  delay(500);
  digitalWrite(D12, HIGH);
  delay(500);
  digitalWrite(D12, LOW);
  //calibration end
}

void loop() {
  ArduinoCloud.update();
  mainrobot();
}

void mainrobot() {
  position = qtr.readLineBlack(sensorValues);
  float error = 2000 - position;
  while (sensorValues[0]>=970 && sensorValues[1]>=970 && sensorValues[2]>=980 && sensorValues[3]>=970 && sensorValues[4]>=970){ 
    if (previousError>0){       //Turn left if the line was to the left before it left the line
      movemotor(-230,230);
    }
    else {
      movemotor(230,-230); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);

    float PID(float error);
  }
}

void movemotor(int left, int right) {
  if (right > 0){
    motorR.setSpeed(right * RMcorrection);
    motorR.forward();
  }
  else {
    motorR.setSpeed(right * RMcorrection);
    motorR.backward();
  }
  if (left > 0){
    motorL.setSpeed(left * LMcorrection);
    motorL.forward();
  }
  else {
    motorL.setSpeed(left * LMcorrection);
    motorL.backward();
  }
}

float PID(float error) {   //PID algorithm function. 
  float previousError = error;

  //PID algorithm variables
  float P = error;    //proportional term: does a lot of the heavy lifting. other calculus-based variables are for precision
  float I = I + error;    //integral term: identifies underlying error that could build up. 
  float D = error - previousError;     //derivative term: identifies rate of change of the error, so that later on we don't overshoot and oscillate around the target value
  float Pvalue = (Kp / pow(10, multiP)) * P; //scaling here makes the constant values easier to be adjusted accurately
  float Ivalue = (Ki / pow(10, multiI)) * I; //the arithmetic here multiplies the constant by the error variables (see https://w.wiki/8Wqd wikipedia's page on it)
  float Dvalue = (Kd / pow(10, multiD)) * D;


  PIDvalue = Pvalue + Ivalue + Dvalue;
  lspeedfinal = lspeed + PIDvalue;
  rspeedfinal = lspeed - PIDvalue;

  if (lspeedfinal > 255){ //prevents overshoot
    lspeedfinal = 255;
  }  
  if (lspeedfinal < -255){
    lspeedfinal = -255;
  }
  if (rspeedfinal > 255) {
    rspeedfinal = 255;
  }
  if (rspeedfinal  < -255) {
    rspeedfinal = -255;
  }
  movemotor(lspeedfinal, rspeedfinal);
  return PIDvalue;
}




void onKdadjChange()  {
  Kd = Kdadj;
}

void onKiadjChange()  {
  Ki = Kiadj;
}


void onKpadjChange()  {
  Kp = Kpadj;
}
