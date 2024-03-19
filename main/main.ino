#include <Wire.h>
#include <Arduino.h>

#include "motor_controls.h"
#include "distance.h"
#include "gyro.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Begin!{}{}{}{}");
  Wire.begin(); // start I2C things

  // Disable i2c pullup resistors :
  https://forum.arduino.cc/t/internal-pull-up-resistor-in-i2c-arduino-uno/977853/17
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  motors_init();

  dist_sensor_init();
  // gyro_init();
  Serial.println("got here1.");

  dist_pid_init(&dist_pid);
  // gyro_pid_init(&gyro_pid);
  Serial.println("got here.");

}

unsigned long int last_loop = 0;
void loop() {
  Serial.println("got here. 2");

  // gyro_pid.update(&gyro_pid); 
  dist_pid.update(&dist_pid);

  // print_pid(&gyro_pid, "gyro");
  // print_pid(&dist_pid, "dist");

  trans_pow = dist_pid.u;
  // // rot_pow = gyro_pid.u;
  update_motors();
}
