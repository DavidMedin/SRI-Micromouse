#include <OPT3101.h>
#include <Wire.h>
#include <Arduino.h>
#include <MPU9250.h>
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

void print_pid(PID* pid) {
  Serial.print(",e:");
  Serial.print(pid->e);

  Serial.print(",int_e:");
  Serial.print(pid->int_e);

  Serial.print(",d_e:");
  Serial.print(pid->d_e);

  Serial.print(",u:");
  Serial.print(pid->u);
}

PID distance_pid;

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




//https://github.com/hideakitai/MPU9250
// DLPF : Digital Low Pass Filter
//https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
MPU9250 mpu;
unsigned long int gyro_before_time = 0;
float rotation = 0;
float bias = 0;
float sample_gyro_z(int sample_count) {
  //const sample_count = 10;
  // static int buff[sample_count];
  float avg = 0;
  // float bias = mpu.getGyroBiasZ(); 
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

PID gyro_pid;




void pid_init() {
  // Initialize Distance Sensor<->Motor PID.
  if(dist_data_ready){
    sensor.readOutputRegs();
    dist_data_ready = false;
    distance_pid.last_time = millis();
    distance_pid.setpoint = 200;
    distance_pid.last_e = sensor.distanceMillimeters - distance_pid.setpoint; // init the derivative.
    distance_pid.K_p = 7.0;
    distance_pid.K_i = 0.2;
    distance_pid.K_d = 0.5;
  }

  // Initialize Gyro<->Motor PID.
  // gyro_pid.last_e = 
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
  gyro_pid.last_time = millis();
  gyro_pid.int_e = 0;

}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // start I2C things
  https://forum.arduino.cc/t/internal-pull-up-resistor-in-i2c-arduino-uno/977853/17
  // Disable i2c pullup resistors
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  motors_init();

  dist_sensor_init();

  gyro_init();

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


  // if(mpu.available()) {
  //   mpu.update_accel_gyro();

    // float gyro_x = mpu.getGyroX();
    // float gyro_y = mpu.getGyroY();
  float gyro_z = sample_gyro_z(100);

  unsigned long int now = millis();
  unsigned long int dt = now - gyro_before_time;
  gyro_before_time = now;

  rotation += gyro_z * (float)dt;

  // Serial.print("gyro_x:");
  // Serial.print(gyro_x);
  // Serial.print(",gyro_y:");
  // Serial.print(gyro_y);
  Serial.print("gyro_z:");
  Serial.print(gyro_z);
  // Serial.print(",");

  Serial.print(",rotation:");
  Serial.print(rotation);
  Serial.print(",");
  // }

  // if(mpu.available()) {
  //   mpu.update_accel_gyro();

    // float gyro_x = mpu.getGyroX();
    // float gyro_y = mpu.getGyroY();
  float gyro_z = sample_gyro_z(100);

  unsigned long int now = millis();
  unsigned long int dt = now - gyro_before_time;
  gyro_before_time = now;

  rotation += gyro_z * (float)dt;

  // Serial.print("gyro_x:");
  // Serial.print(gyro_x);
  // Serial.print(",gyro_y:");
  // Serial.print(gyro_y);
  Serial.print("gyro_z:");
  Serial.print(gyro_z);
  // Serial.print(",");

  Serial.print(",rotation:");
  Serial.print(rotation);
  Serial.print(",");
  // }

  distance_pid.update(&distance_pid);
  Serial.println();
}
