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
int speed = 1023-100;
int acc = 100;
int dir = 1;

// *in is the input from the motor to the computer.
//        PWM | In  | Dir
// right : 10    8    11 
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
int dist_sample_ready_pin = 1;

// Number of samples the sensor does per sensor output.
int dist_sensor_sample_count = 500;
// Estimated max time a sensor output takes in milliseconds.
int dist_sensor_time_max = (int) ( (float)( dist_sensor_sample_count * 25 /*ms*/ ) * 1.06 );

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

  sensor.setFrameTiming(500); 
  // 500 samples * 0.25 ms + 6% = whatevre
  // 256 samples * 0.25 ms + 6% = 68 ms
  sensor.setChannel(1); // TX1, middle channel

  sensor.setBrightness(OPT3101Brightness::Adaptive);
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // unsigned long int ahhh = millis();
  // Serial.print(ahhh);
  // Serial.println();
  // Serial.print((unsigned long int)((float)1000*((float)millis() / (float)1000)));
  // Serial.println();
  // Serial.print((float)millis() / (float)1000)
  sensor.enableDataReadyOutput(2);
  // sensor.enableTimingGenerator();
  // sensor.setContinuousMode();
  sensor.startSample();
  // sensor.startSample();

  acc_x_offset = analogRead(acc_x_pin);
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


// Motor Signal State
unsigned long int mspr = 0; // milliseconds per rotation
int last_rot_sig = 0; // last value recieved from the motor.
unsigned long int time_ms_of_last_sig = millis();
// ===


// Distance Sensor State
int16_t last_dist = 0;

float mill_p_msecond_vel;
float mmps;
// ========


// long int vel_x_g = 0;
unsigned long int last_loop = 0;
void loop() {
  // sensor.sample();


  // ============ vvvvvvvvvvvv ========== Reading Motor 1/9th signals to (poorly) determine speed.
  // calculate rotations per second.
  // int r_sig_motor = digitalRead(RMotor_In);
  // unsigned long int time_now = millis(); 
  // mspr = (time_now-time_ms_of_last_sig) * 9;
  // if(r_sig_motor != last_rot_sig) {
  //   // +1 1/9 rotation.
    
  //   time_ms_of_last_sig = time_now;
  //   last_rot_sig = r_sig_motor;
  // }

  // Serial.print( (1000.0 / mspr) );
  // Serial.print( mspr);
  // Serial.println();

  // ================== ^^^^^^^^^^^^^^^=====================

  // ============= vvvvvvvvvvvvvvv ============= Distance Sensor

  // == Use sensor 3 (the forward sensor) to guess velocity.
  int is_ready = sensor.isSampleDone(); // digitalRead(dist_sample_ready_pin);
  if(is_ready == 1) {
    sensor.readOutputRegs();
    sensor.startSample();

    // read time
    unsigned long int this_loop = millis();
    unsigned long int dt = this_loop - last_loop; // dt = delta time, the time since the last loop.
    last_loop = this_loop;

    // interpret data.
    // The distance traveled - in mm - since the last sensor output. 
    int distance_traveled = last_dist - sensor.distanceMillimeters;
    mill_p_msecond_vel = ( (float)distance_traveled ) / (float)dt;
    last_dist = sensor.distanceMillimeters;
    mmps = (float)mill_p_msecond_vel * 1000.0; // 1000.0

    // Serial.print("vel:");
    // Serial.print(mmps);
    // Serial.print(",");
    // Serial.print("dist:");
    // Serial.print(sensor.distanceMillimeters);
    // Serial.print(",");
    // Serial.print("going:");
    // Serial.print(going);
    // Serial.println("");
      // Simple state machine.
      if(going == true) { // going forward?
        //  try to predict if the robot will be at or pass the target distance (100 mm).
        int dist_to_wall = sensor.distanceMillimeters;
        int predict_dist_next_sample = dist_to_wall - distance_traveled;
        Serial.print("Distance to wall : ");
        Serial.println(dist_to_wall);
        Serial.print("Predicted distance : ");
        Serial.println(predict_dist_next_sample);

        if(predict_dist_next_sample <= 400) { // If close enough to wall.
          going = false; // try to stop.
          flip_motor_dir(); // try to go backwards.
        }
      }else if(going == false) { // going backwards. Just being explicit.
        float tolerance = 20.0;
        if(mmps < tolerance) {
          // Looks like we are sufficiently stopped.
          analogWrite(LMotor_PWM, NO_SPEED); // try to stop.
          analogWrite(RMotor_PWM, NO_SPEED);
        }

        if(sensor.distanceMillimeters > 200) { // If far enough away...
          going = true; // go forward.
          fwd_motor_dir(); 
        }
      }
  }


  // Serial.println(is_ready);
  
  // if(sensor.distanceMillimeters <= 100 ) {
  //   analogWrite(LMotor_PWM, NO_SPEED);
  //   analogWrite(RMotor_PWM, NO_SPEED);
  //   flip_motor_dir();
  //   analogWrite(LMotor_PWM, speed);
  //   analogWrite(RMotor_PWM, speed);
  //   delay(100);
  //   analogWrite(LMotor_PWM, NO_SPEED);
  //   analogWrite(RMotor_PWM, NO_SPEED);
  //   flip_motor_dir();
  // }
  // else {
  //   analogWrite(LMotor_PWM, speed);
  //   analogWrite(RMotor_PWM, speed);
  // }

  // ============= ^^^^^^^^^^ ========================================

  // ======================== vvvvvvvvvvvvvvvvvvvvvv ======= Bad Accelerometer Code
  // int acc_x = analogRead(acc_x_pin);
  // int acc_y = analogRead(acc_y_pin);
  // int acc_z = analogRead(acc_z_pin);

  // int acc_x_g = acc_x - acc_x_offset;
  // vel_x_g += acc_x_g * dt;

  // float acc_x_mps2 = acc_x_g / 9.8;
  // float vel_x_mps2 = vel_x_g / 9.8;
  // // float acc_x = analogRead(acc_x_pin)/1023.0;
  // // float acc_y = analogRead(acc_y_pin)/1023.0;
  // // float acc_z = analogRead(acc_z_pin)/1023.0;
  
  // Serial.print("Acc_X:");
  // Serial.print(acc_x_g);
  // Serial.print(",");
  // Serial.print("Vel_X:");
  // Serial.print(vel_x_g);
  // //   Serial.print("Y:");
  // // Serial.print(acc_y);
  // // Serial.print(",");
  // //   Serial.print("Z:");
  // // Serial.print(acc_z);
  // Serial.println();

  // ===================== ^^^^^^^^^^^^^^^^^^^^^^^^^^^ =============================

  // Serial.print(sensor.distanceMillimeters);
  // Serial.println();
  delay(50);
}
