void setup() {
  // D9 - D10 15.6 kHz 10 bit
  TCCR1A = 0b00000011;  // 10 bit
  TCCR1B = 0b00001001;  // x1 fast pwm mode

  //analogWrite(9, 745);
  analogWrite(10, 745);
  analogWrite(9, 745);
}

void loop() {
}
