#include <QTRSensors.h>  //install library
QTRSensors qtr; //makes it easier to type funcs 

const uint8_t SensorCount = 5;  //declare number of sensors in the format the library likes
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float error;  //declare variables 
float PIDvalue;  //these are the main ones which the RPi will need

void setup() {
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

float PID(float error) {   //PID algorithm function. 
  float previousError = error;

  //PID algorithm variables
  float P = error;    //proportional term: does a lot of the heavy lifting. other calculus-based variables are for precision
  float I = I + error;    //integral term: identifies underlying error that could build up. 
  float D = error - previousError;     //derivative term: identifies rate of change of the error, so that later on we don't overshoot and oscillate around the target value
  
  float Kp; //PID constants
  float Ki;
  float Kd;

  float multiP; //scaling variables
  float multiI;
  float multiD;

  float Pvalue = (Kp / pow(10, multiP)) * P; //scaling here makes the constant values easier to be adjusted accurately
  float Ivalue = (Ki / pow(10, multiI)) * I; //the arithmetic here multiplies the constant by the error variables (see https://w.wiki/8Wqd wikipedia's page on it)
  float Dvalue = (Kd / pow(10, multiD)) * D;


  PIDvalue = Pvalue + Ivalue + Dvalue;
  return PIDvalue;
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);  //returns value of 1000-5000, where 1000 means the line is directly below the left-most sensor, and  5000 the same for the right-most sensor. 
  float error = 3000 - position; //We want it directly below the middle sensor, so the 3rd one
  
  float PID(float error); 
  
  //Serial.println("Line's position is: ");  //only for debugging 
  Serial.println(position); //sends position values to RPi using serial , which is connected via usb 
  
  //Serial.println("Error is: ");   //only for debugging 
  Serial.println(error);  //ditto but for error
  
  //Serial.println("PID value is: ");  `   //only for debugging 
  Serial.println(PIDvalue); //final value to be used for steering
  
  //Serial.println("##########################################") // only for debugging: just a separator to make it visually easier to read each data point
  delay(250); //prevents serial monitor going ballistic
}

/** 
Still to do:
- Bluetooth capability
- App to be able to tune Kp, Ki, Kd values
- RPi side motor control.
*/
