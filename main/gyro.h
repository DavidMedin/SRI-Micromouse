#pragma once
#include <Wire.h>
#include <Arduino.h>
#include <MPU9250.h>
#include "pid.h"

//https://github.com/hideakitai/MPU9250
// DLPF : Digital Low Pass Filter
//https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
MPU9250 mpu;
unsigned long int gyro_before_time = 0;
float bias = 0;
float sample_gyro_z(int sample_count) {
  float avg = 0;
  for(int i = 0;i < sample_count;i++){
    while(!mpu.available()) {}
    mpu.update_accel_gyro();
    float reading = mpu.getGyroZ() - bias;
    avg += reading / sample_count;
  }
  return avg;
}

void gyro_init(){
  delay(1000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
  
}

float rotation = 0;
PID gyro_pid;

void gyro_pid_init(PID* self){
    // Initialize Gyro<->Motor PID.

    mpu.verbose(true);
    Serial.print("Before Gyro z bias : ");
    Serial.println(mpu.getGyroBias(0));
    Serial.println(mpu.getGyroBias(1));
    Serial.println(mpu.getGyroBias(2));
    // mpu.calibrateAccelGyro();
    Serial.print("After Gyro z bias : ");
    Serial.println(mpu.getGyroBias(0));
    Serial.println(mpu.getGyroBias(1));
    Serial.println(mpu.getGyroBias(2));
    bias = sample_gyro_z(100);
    // mpu.setGyroBias(0, 0, bias);

    // mpu.print_calibration();
    mpu.verbose(false);
    delay(1000);
    self->last_time = millis();
    self->setpoint = 0; // TODO: Make this correct.
    self->int_e = 0;
}

void gyro_pid_update(PID* self){

    // read time
    unsigned long int this_time = millis(); // milliseconds
    float dt = ( (float)(this_time - self->last_time) / 1000.0 ) ; // dt = delta time, the time since the last loop.
    self->last_time = this_time;

    // Update the 'rotational position' from the 'rotational velocity'.
    float rot_vel = sample_gyro_z(100);
    rotation += rot_vel * dt;

    // PID Loop:

    // P: Proportional
    // I: Integral (accumulate)
    // D: Derivative (difference)

    // e(t) = 
    // The difference between the target and the current.
    // For us, that is the difference between our current distance to the wall, and 200 mm from the wall.
    float e = rotation - self->setpoint;
    self->int_e += e * dt;
    self->int_e = constrain(self->int_e, (float)-INT16_MAX, (float)INT16_MAX);

    self->d_e = (e - self->last_e) / dt;
    self->last_e = e;

    //u(t) = 
    // The resulting 'power' of the motors. Also encodes direction.
    // u(t) = K_p * e(t)  +  K_i * Int_0_t e(t)  +  [ K_d * d*e(t) ]/dt
    float u = self->K_p * e   + self->K_i * self->int_e  +  self->K_d * self->d_e;
    self->u = constrain(u, -(float)INT16_MAX, (float)INT16_MAX);
    self->e = (float)e; // for logging.
}