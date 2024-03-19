#pragma once
#include <Arduino.h>
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
int speed = NO_SPEED;
int acc = 100;
//int dir = 1;

// *in is the input from the motor to the computer.
//        PWM | In  | Dir
// right : 10   11    8 
// left :  9    12    13

int RMotor_PWM = 10;
int LMotor_PWM = 9;

int RMotor_Dir_Pin = 8;
int LMotor_Dir_Pin = 13;
bool RMotor_Dir = false;
bool LMotor_Dir = true;

int RMotor_In = 11;
int LMotor_In = 12;

void motors_init(){
  // Set register magic to allow PWM to go faster than it should normally.
  // D9 - D10 15.6 kHz 10 bit
  TCCR1A = 0b00000011;  // 10 bit
  TCCR1B = 0b00001001;  // x1 fast pwm mode

  // Tell motors to go speedy through PWM.
  analogWrite(LMotor_PWM, speed);
  analogWrite(RMotor_PWM, speed);

  // Go 'forward'
  pinMode(RMotor_Dir_Pin, OUTPUT);
  pinMode(LMotor_Dir_Pin, OUTPUT);
  digitalWrite(RMotor_Dir_Pin, RMotor_Dir);
  digitalWrite(LMotor_Dir_Pin, LMotor_Dir);
}


void flip_motor_dir() {
  RMotor_Dir = !RMotor_Dir;
  LMotor_Dir = !LMotor_Dir;
  digitalWrite(RMotor_Dir_Pin, RMotor_Dir);
  digitalWrite(LMotor_Dir_Pin, LMotor_Dir);
}

void fwd_motor_dir() {
  RMotor_Dir = false;
  LMotor_Dir = true;
  digitalWrite(RMotor_Dir_Pin, RMotor_Dir);
  digitalWrite(LMotor_Dir_Pin, LMotor_Dir);
}

void bwd_motor_dir() {
  RMotor_Dir = true;
  LMotor_Dir = false;
  digitalWrite(RMotor_Dir_Pin, RMotor_Dir);
  digitalWrite(LMotor_Dir_Pin, LMotor_Dir);
}

// Power referes to 'effort', and direction.
// 1023 is full 'power' (attempt max speed), 0 is 'no power',
// and -1023 is full power in the opposite direction.
void set_motor_power(int power) {
    const int max_power = 300; // nice
    int bounded_power = constrain( power, -max_power, max_power);
    // Serial.print(",bounded_power:");
    // Serial.print(power);
    bounded_power >= 0 ? fwd_motor_dir() : bwd_motor_dir();
    int abs_speed = ( -1 * ( abs(bounded_power) - 500 ) ) + 500;

    analogWrite(RMotor_PWM, abs_speed );
    analogWrite(LMotor_PWM, abs_speed );
    // Serial.print(",motors:");
    // Serial.print(abs_speed);
    // Serial.print(",dir:");
    // Serial.print(LMotor_Dir);
}
