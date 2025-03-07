#include "thingProperties.h" //i did not include this in the github for security reasons
#include <QTRSensors.h>  //install library

QTRSensors qtr; //makes it easier to type funcs 

float lspeed; //initial speed variables
float rspeed;
float lspeedfinal;//final adjusted speed variables for motors
float rspeedfinal;

float Kd; //final pid gain values
float Ki;
float Kp; 
float PIDvalue;  

float multiP = 0.0;  //scaling vals: can be changed to make constant adjustment less sensitive.
float multiI = 0.0;
float multiD = 0.0;

const uint8_t SensorCount = 5;  //declare number of sensors in the format the library likes
uint16_t sensorValues[SensorCount]; 
int threshold[SensorCount]; //other setup functions

unsigned long startMillis;  //for timing purposes. using delay() would mess up the board's timing + cloud updates
unsigned long currentMillis;
const unsigned long period = 50;  //ms 

uint16_t position;  
float previousError; //for derivative gain value

void setup() {
  initProperties(); //initialise values from thingproperties.h

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();  //sends info to debug monitor 
  //motor controller pins
  pinMode(2, OUTPUT); //ENA
  pinMode(3, OUTPUT); //IN1
  pinMode(4, OUTPUT); //IN2

  pinMode(5, OUTPUT); //ENB
  pinMode(6, OUTPUT); //IN1
  pinMode(7, OUTPUT); //IN2
  
  //hbridge control
  digitalWrite(3, HIGH);  //set motors to forward always: we won't ever need to go backwards. 
  digitalWrite(4, LOW);

  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);

  qtr.setTypeAnalog();//initialise sensors. 
  qtr.setSensorPins((const uint8_t[]){ A1, A2, A3, A4, A5 }, SensorCount);
  pinMode(D12, OUTPUT); //for leds to check when calibration is finished
  pinMode(D13, OUTPUT);
  
  while (ArduinoCloud.connected() == 0)  {
    digitalWrite(D13, HIGH);
    delay(500);
  }
  while (ArduinoCloud.connected() == 1){ //if connected
    digitalWrite(D13, LOW);// turn off LED and 
        
    //start calibration
    digitalWrite(D12, HIGH);  //turn on led, connected to digital pin 12, to indicate we are calibrating
    for (uint16_t i = 0; i < 250; i++) //repeat 250 times.
    {
      currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
      if (currentMillis - startMillis >= period)  //test whether the period has elapsed
        {
          qtr.calibrate();  //must ensure that sensors are placed at height which will be used for the line follow sequence
          startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
    }
  }
  digitalWrite(D12, LOW);
  //end calibration
  }
}


void loop() {
  ArduinoCloud.update();  // send vals back and forth 
  mainrobot();
}


void Lmove(float lspeed) {   //prevents overshoot. basic motor functions 
  if (lspeed > 255) {
    lspeed = 255;
  }
  if (lspeed < -255) {
    lspeed = -255;
  }
  analogWrite(5, lspeed);
}

void Rmove(float rspeed) {
  if (rspeed > 255) {
    rspeed = 255;
  }
  if (rspeed < -255) {
    rspeed = -255;
  }
  analogWrite(2, rspeed);
}

void movemotor(float lspeed, float rspeed) {
  Lmove(lspeed);
  Rmove(rspeed);
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

  movemotor(lspeedfinal, rspeedfinal);
  return PIDvalue; //for debugging if needed
}


void mainrobot() {
  position = qtr.readLineBlack(sensorValues);  //read values
  float error = 2000 - position;   
  while (sensorValues[0]>=970 && sensorValues[1]>=970 && sensorValues[2]>=980 && sensorValues[3]>=970 && sensorValues[4]>=970){ 
    if(previousError>0){       //Turn left if the line was to the left before it deviated from the line
      movemotor(-230,230);
    }
    else{
      movemotor(230,-230); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);

    float PID(float error);
  }
}

//update read/write cloud variables
void onKdadjChange()  {
  Kd = Kdadj;
}
void onKiadjChange()  {
  Ki = Kiadj;
}
void onKpadjChange()  {
  Kp = Kpadj;
}
