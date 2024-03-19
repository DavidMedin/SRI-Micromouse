#pragma once
#include <Arduino.h>
#define freak_out(motor) if( !(motor == 0 || motor == 1) ){ Serial.println("\nError! Bad motor usage!"); }
/*
Motor Magic:
Black : ground
Red : 12v power
Yellow : Signal out, 9 pulse signals per revolution.
White : Duty Cycle for pwm 15~25kHz
Orange : Forward / Reverse. Overcuirrent protection is 0.8A.
  If that current of 0.8A is reached, protection is enabled
  and needs to be re-energized to start again.
*/
//0 (max speed) (0v) - 1023 (min speed) (5v)
const int NO_SPEED = 1023;
const int ALL_SPEED = 0;

// *in is the input from the motor to the computer.
//        PWM | In  | Dir
// right : 10   11    8 
// left :  9    12    13


// [left, right]
int motors_pwm_pin[2] = {9, 10};
int motors_dir_pin[2] = {13, 8};
bool motors_dir[2] = {true, false};


void flip_motor_dir(int motor) {
  freak_out(motor);
  motors_dir[motor] = !motors_dir[motor];
  digitalWrite(motors_dir_pin[motor], motors_dir[motor]);
}
void flip_motors_dir() {
  flip_motor_dir(0);
  flip_motor_dir(1);
}

void fwd_motor_dir(int motor){
  freak_out(motor);

  if(motor == 0)
    motors_dir[0] = true;
  else
    motors_dir[1] = false;

  digitalWrite(motors_dir_pin[motor], motors_dir[motor]);
}
void bwd_motor_dir(int motor){
  freak_out(motor);

  if(motor == 0)
    motors_dir[0] = false;
  else
    motors_dir[1] = true;

  digitalWrite(motors_dir_pin[motor], motors_dir[motor]);
}
void fwd_motors_dir(){
  fwd_motor_dir(0);
  fwd_motor_dir(1);
}
void bwd_motors_dir(){
  bwd_motor_dir(0);
  bwd_motor_dir(1);
}

const int max_power = 300; // nice
int trans_pow = 0; // -1023 to 1023. May be limited by update_motors() [ like if we have a max_power ].
int rot_pow = 0; // negative values are for turning left, and positive values are rotating right.
int speeds[2] = {NO_SPEED, NO_SPEED};
void update_motors() {
  int bounded_power_l = constrain( trans_pow - rot_pow, -max_power, max_power);
  int bounded_power_r = constrain( trans_pow + rot_pow, -max_power, max_power);

  bounded_power_l >= 0 ? fwd_motor_dir(0) : bwd_motor_dir(0);
  bounded_power_r >= 0 ? fwd_motor_dir(1) : bwd_motor_dir(1);
  int abs_speed_l = ( -1 * ( abs(bounded_power_l) - 500 ) ) + 500;
  int abs_speed_r = ( -1 * ( abs(bounded_power_r) - 500 ) ) + 500;

  analogWrite(motors_pwm_pin[0], abs_speed_l );
  analogWrite(motors_pwm_pin[1], abs_speed_r );
}

void motors_init(){
  // Set register magic to allow PWM to go faster than it should normally.
  // D9 - D10 15.6 kHz 10 bit
  TCCR1A = 0b00000011;  // 10 bit
  TCCR1B = 0b00001001;  // x1 fast pwm mode

  pinMode(motors_dir_pin[0], OUTPUT);
  pinMode(motors_dir_pin[1], OUTPUT);
  digitalWrite(motors_dir_pin[0], motors_dir[0]);
  digitalWrite(motors_dir_pin[1], motors_dir[1]);

  // Tell motors to stop.
  // set_motor_power(0);
  update_motors();
}
