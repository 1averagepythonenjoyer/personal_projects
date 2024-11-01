#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

Serial.begin(9600)
void setup() {
  // put your setup code here, to run once:
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5}, SensorCount);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //built-in LED on the arduino tells us that the steup is complete. 
}

void loop() {
  // put your main code here, to run repeatedly:

}
