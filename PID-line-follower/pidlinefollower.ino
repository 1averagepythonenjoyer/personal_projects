#include <QTRSensors.h>  //install library
#include "thingProperties.h"
#include <L298N.h>
QTRSensors qtr; //makes it easier to type funcs 

#define EN_A 2
#define IN1_A 3
#define IN2_A 4

#define EN_B 5
#define IN1_B 6
#define IN2_b 7

float RMcorrection = 1.0;
float LMcorrection = 0.95;
float lspeed;
float lspeedfinal;
float rspeedfinal;

const uint8_t SensorCount = 5;  //declare number of sensors in the format the library likes
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float error;  //declare variables 
float PIDvalue;  

float Kp; 
float Ki;
float Kd;

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

  //calibration start
  pinMode(D12, OUTPUT);
  digitalWrite(D12, HIGH);          //turn on led, connected to digital pin 12, to indicate we are calibrating
  for (uint16_t i = 0; i < 400; i++)  //400 repeats, so a few minutes worth of calibration
  {
    qtr.calibrate();  //must ensure that sensors are placed at height which will be used for the line follow sequence
    delay(500);
  }
  digitalWrite(D12, LOW);
  delay(500);
  digitalWrite(D12, HIGH);
  delay(500);
  digitalWrite(D12, LOW);
  //calibration end
  Serial.begin(9600); //begins serial communication with RPi at 9600 baud
  
}

void movemotor(int left, int right) {
  if (right > 0){
    motorR.setSpeed(right * RMcorrection);
    motorR.forward();
  }
  else {
    motorR.setspeed(right * RMcorrection)
    motorR.backward();
  }
  if (left > 0){
    motorL.setspeed(left * LMcorrection);
    motorL.forward();
  }
  else {
    motorL.setspeed(left * LMcorrection);
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

  if (lspeedfinal > 255){
    lspeedfinal = 255;
  }  
  if (lspeedfinal < -255){
    lspeedfinal = -255;
  }
  if (rspeedfinal > 255) {
    rspeedfinal = 255;
  }
  if (rspeedfinal) {
  movemotor(lspeedfinal, rspeedfinal);
  return PIDvalue;
  }
}

void mainrobot() {
  position = qtr.readLineBlack(sensorValues);
  float error = 2000 - position;
  while (sensorValues[0]>=970 && sensorValues[1]>=970 && sensorValues[2]>=980 && sensorValues[3]>=970 && sensorValues[4]>=970){ 
    if(previousError>0){       //Turn left if the line was to the left before it left the line
      movemotor(-230,230);
    }
    else{
      movemotor(230,-230); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);

    float PID(float error);
}

void loop() {
  ArduinoCloud.update();
  mainrobot()
}




/*
  Since Kp is READ_WRITE variable, onKpChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onKpChange()  {
  // Add your code here to act upon Kp change
}
/*
  Since Kd is READ_WRITE variable, onKdChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onKdChange()  {
  // Add your code here to act upon Kd change
}
/*
  Since Ki is READ_WRITE variable, onKiChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onKiChange()  {
  // Add your code here to act upon Ki change
}
