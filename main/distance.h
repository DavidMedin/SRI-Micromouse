#pragma once
#include <OPT3101.h>
#include <Wire.h>
#include <Arduino.h>

#include "pid.h"
// ========== Sensors =================
/*
Cables for I2C of Sensor
Green : data - sda
Blue : clock - scl
*/
OPT3101 sensor;
//The pin the sensor *should* output a pulse on when a sample is ready.
int dist_sample_ready_pin = 2;

// Number of samples the sensor does per sensor output.
int dist_sensor_sample_count = pow(2,5);


PID dist_pid;

volatile bool dist_data_ready = false;
void dist_data_ready_func() {
    dist_data_ready = true;
}


void dist_sensor_init() {
  pinMode(dist_sample_ready_pin, INPUT);

  // Sensor Initialization vvvvvvvvvvvvvvvvv
  sensor.init();
  // yoink : https://github.com/pololu/opt3101-arduino/blob/master/examples/Basic/Basic.ino
  if(sensor.getLastError() != 0) {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {}
  }

  sensor.setFrameTiming(dist_sensor_sample_count); 
  // 500 samples * 0.25 ms + 6% = whatevre
  // 256 samples * 0.25 ms + 6% = 68 ms
  sensor.setChannel(1); // TX1, middle channel

  sensor.setBrightness(OPT3101Brightness::Adaptive);
  sensor.setContinuousMode();
  sensor.enableDataReadyOutput(1);

  attachInterrupt(digitalPinToInterrupt(dist_sample_ready_pin), dist_data_ready_func, RISING);
  sensor.enableTimingGenerator();
}

void dist_pid_init(PID* self){
    // Initialize Distance Sensor<->Motor PID.
  Serial.println("got here5.");

    while(!dist_data_ready){}
    Serial.println("got here6.");
    dist_data_ready = false;
    delay(1000);
    sensor.readOutputRegs();
    Serial.println("got here8.");
    Serial.println("got here7.abababab");
    dist_data_ready = false;

    dist_pid.last_time = millis();
    Serial.println("got here9.");
    dist_pid.setpoint = 200;
    dist_pid.last_e = sensor.distanceMillimeters - dist_pid.setpoint; // init the derivative.
    dist_pid.K_p = 7.0;
    dist_pid.K_i = 0.2;
    dist_pid.K_d = 0.5;
    Serial.println("got here10.");
}

void wall_dist_update(PID* self) {
  if(dist_data_ready) {

  sensor.readOutputRegs();
  // read time
  unsigned long int this_time = millis(); // milliseconds
  float dt = ( (float)(this_time - self->last_time) / 1000.0 ) ; // dt = delta time, the time since the last loop.
  self->last_time = this_time;

  // PID Loop:

  // P: Proportional
  // I: Integral (accumulate)
  // D: Derivative (difference)

  // e(t) = 
  // The difference between the target and the current.
  // For us, that is the difference between our current distance to the wall, and 200 mm from the wall.
  int e = sensor.distanceMillimeters - self->setpoint;
  dist_data_ready = false;
  self->int_e += e * dt;
  self->int_e = constrain(self->int_e, -INT16_MAX, INT16_MAX);

  self->d_e = ((float)e - self->last_e) / dt;
  self->last_e = e;

  //u(t) = 
  // The resulting 'power' of the motors. Also encodes direction.
  // u(t) = K_p * e(t)  +  K_i * Int_0_t e(t)  +  [ K_d * d*e(t) ]/dt
  float u = self->K_p * (float)e   + self->K_i * self->int_e  +  self->K_d * self->d_e;
  self->u = constrain(u, -(float)INT16_MAX, (float)INT16_MAX);
  self->e = (float)e; // for logging.

  // set_motor_power((int)u);

  }
}

