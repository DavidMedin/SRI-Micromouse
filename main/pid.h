#pragma once

struct PID {
  float K_p = 1.0;
  float K_i = 1.0;
  float K_d = 1.0;
  float last_time;
  float setpoint;
  float int_e=0;
  float last_e;
  float u=0;

  float e;
  float d_e;
  void (*update)(PID*);
};

void print_pid(PID* pid, const char* prefix) {
    Serial.print(",");
    Serial.print(prefix);
    Serial.print("_e:");
    Serial.print(pid->e);

    Serial.print(",");
    Serial.print(prefix);
    Serial.print("_int_e:");
    Serial.print(pid->int_e);

    Serial.print(",");
    Serial.print(prefix);
    Serial.print("_d_e:");
    Serial.print(pid->d_e);
    
    Serial.print(",");
    Serial.print(prefix);
    Serial.print("_u:");
    Serial.print(pid->u);
}