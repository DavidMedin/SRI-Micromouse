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
//0 (max speed) - 1023 (min speed)
int speed = 1023-100;
int acc = 100;
int dir = 1;

int LMotor_PWM = 10;
int RMotor_PWM = 9;

int LMotor_Dir = 6;
int RMotor_Dir = 5;

void setup() {
  Serial.begin(115200);
  // D9 - D10 15.6 kHz 10 bit
  TCCR1A = 0b00000011;  // 10 bit
  TCCR1B = 0b00001001;  // x1 fast pwm mode

  //analogWrite(9, 745);
  analogWrite(LMotor_PWM, speed);
  analogWrite(RMotor_PWM, speed);

  pinMode(LMotor_Dir, OUTPUT);
  pinMode(RMotor_Dir, OUTPUT);
  digitalWrite(LMotor_Dir, HIGH);
  digitalWrite(RMotor_Dir, LOW);
}

void loop() {
  // if(speed+dir*acc > 1023 || speed+dir*acc < 0 ) {
  //   dir = dir*-1;
  // }

  // speed+=dir * acc;
  // Serial.println( speed );
  // analogWrite(10, speed);
  // analogWrite(9, speed);
  // delay(500);
}
