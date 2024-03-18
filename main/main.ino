#include <OPT3101.h>
#include <Wire.h>
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
// Estimated max time a sensor output takes in milliseconds.
// int dist_sensor_time_max = (int) ( (float)( dist_sensor_sample_count * 25 /*ms*/ ) * 1.06 );

// Accelerometer pins (the order might be wrong).
int acc_x_pin = A0;
int acc_y_pin = A1;
int acc_z_pin = A2;

int acc_x_offset = 0;
// Input accelormeter voltage is 3.3 V, so output voltage is sensitive to
// ~330 mV/g.
// zero g is about 3.3/2.
// so, analogRead() should return around 337-338 for zero.
// 250?
// ====================================


float K_p = 7.0;
float K_i = 0.2; // 2
float K_d = 0.5; // 1

int32_t target_distance = 200;

float int_e = 0;
int16_t last_error = 0;

volatile bool data_ready = false;
void data_ready_func() {
    data_ready = true;
}

void pid_init() {
  if(data_ready){
    sensor.readOutputRegs();
    data_ready = false;

    last_error = sensor.distanceMillimeters - target_distance; // init the derivative.
  	int_e = 0; // init the integral
  }

}


void setup() {
  Serial.begin(115200);
  Wire.begin(); // start I2C things

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

  attachInterrupt(digitalPinToInterrupt(dist_sample_ready_pin), data_ready_func, RISING);
  sensor.enableTimingGenerator();
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  pid_init();
}

// ================= Robot State =========
bool going = true; // she's going! Don't stop!

// ===============


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
    Serial.print(",bounded_power:");
    Serial.print(power);
    bounded_power >= 0 ? fwd_motor_dir() : bwd_motor_dir();
    int abs_speed = ( -1 * ( abs(bounded_power) - 500 ) ) + 500;

    analogWrite(RMotor_PWM, abs_speed );
    analogWrite(LMotor_PWM, abs_speed );
    Serial.print(",motors:");
    Serial.print(abs_speed);
    Serial.print(",dir:");
    Serial.print(LMotor_Dir);
}

unsigned long int last_loop = 0;
void loop() {


    if(data_ready) {

      sensor.readOutputRegs();
      // read time
      unsigned long int this_loop = millis(); // milliseconds
      float dt = ( (float)(this_loop - last_loop) / 1000.0 ) ; // dt = delta time, the time since the last loop.
      last_loop = this_loop;

      // PID Loop:

      // P: Proportional
      // I: Integral (accumulate)
      // D: Derivative (difference)


      // e(t) = 
      // The difference between the target and the current.
      // For us, that is the difference between our current distance to the wall, and 200 mm from the wall.
      int e = sensor.distanceMillimeters - target_distance;
      data_ready = false;
      int_e += e * dt;
      int_e = constrain(int_e, -INT16_MAX, INT16_MAX);

      float d_e = ((float)e - (float)last_error) / dt;
      last_error = e;

      //u(t) = 
      // The resulting 'power' of the motors. Also encodes direction.
      // u(t) = K_p * e(t)  +  K_i * Int_0_t e(t)  +  [ K_d * d*e(t) ]/dt
      float u = K_p * (float)e   + K_i * int_e  +  K_d * d_e;
      u = constrain(u, -(float)INT16_MAX, (float)INT16_MAX);
      Serial.print("helpme:");
      Serial.print(sensor.distanceMillimeters);

      Serial.print(",dt:");
      Serial.print(dt);

      Serial.print(",e:");
      Serial.print(e);

      Serial.print(",int_e:");
      Serial.print(int_e);

      Serial.print(",d_e:");
      Serial.print(d_e);

      set_motor_power((int)u);
      Serial.print(",u:");
      Serial.print(u);
      Serial.println();

    }

}
