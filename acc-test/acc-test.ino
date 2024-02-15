#include <ADXL335.h>

ADXL335 accelerometer;
float accX;
 

void setup() {
  accelerometer.begin();

  Serial.begin(9600);
}

void loop() {
  accX = accelerometer.getAccelerationX();
  delay(100);
  Serial.print("Acc X: ");
  Serial.println(accX);
}
