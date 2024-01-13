#include "Arduino_LED_Matrix.h"
#include <FspTimer.h>
#define u32_t uint32_t

#define MAX_INT 0xffffffff
#define TOP_ROW MAX_INT<<20 // The mask of the first number in a matrix frame for the top row.
#define STRIPED_TOP_ROW 0b101010101010<<20

// uint32_t rows[] = {
// MAX_INT<<20,
// MAX_INT<<8,
// };

// int battery_mon_pins[] = {A0,A1};

int MotorPWM = D6;
int MotorDir = D4;

int analog_pin = A0;
ArduinoLEDMatrix matrix;
unsigned long canvas[3] = {
  0,0,0
  // 0b00000000000000000000000000000000,
  // 0b00000000000000000000000000000000,
  // 0b00000000000000000000000000000000,
};

// void set_pix(uint32_t[3] canvas, uint32_t x, uint32_t y, byte value) {

// }

// void set_row(uint32_t[3] canvas, uint32_t row, uint16_t value) {
//   uint32_t r = (row*12) / 32;

//   //            value in its place   with  the other row(s), but only them.
//   canvas[r] = ((uint32_t)value<<offset) | (canvas[r]&(!row_mask));
// }

FspTimer audio_timer;
uint64_t count=0;
uint64_t start_time=0;

// callback method used by timer
void timer_callback(timer_callback_args_t __attribute((unused)) *p_args) {
  count++;
}

bool beginTimer(float rate) {
  uint8_t timer_type = GPT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0){
    tindex = FspTimer::get_available_timer(timer_type, true);
  }
  if (tindex < 0){
    return false;
  }

  FspTimer::force_use_of_pwm_reserved_timer();
  audio_timer.begin()
  if(!audio_timer.begin_pwm(timer_type, tindex,CHANNEL_A)){
    return false;
  }

  if (!audio_timer.setup_overflow_irq()){
    return false;
  }

  if(!audio_timer.set_duty_cycle(500, CHANNEL_A)){
    return false;
  }


  if(!audio_timer.enable_pwm_channel( CHANNEL_A)){
    return false;
  }

  if (!audio_timer.open()){
    return false;
  }

  if (!audio_timer.start()){
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  matrix.begin();

  pinMode(MotorPWM, OUTPUT);

  BeginTimer(1231243);
  // pinMode(MotorDir, OUTPUT);
}

u32_t charge; // index into the top row representing the charge of a battery.
int digitalValue = 0;
float analogVolts;



void loop() {
  // put your main code here, to run repeatedly:
  digitalValue = analogRead(analog_pin);
  analogVolts = digitalValue * 5 / 1023;
  // Serial.print("analog value = ");
  Serial.println(analogVolts );


  charge = (u32_t)((analogVolts-3.7)/0.5 * 12);
  // Serial.print("led_index = ");
  // Serial.println((u32_t)charge );

  if(charge == 0 || charge > 12) {
    // battery is dead
    canvas[0] = STRIPED_TOP_ROW;
  }else {
    // battery is not dead
      canvas[0] = TOP_ROW<<(12-(u32_t)charge);
  }
  matrix.loadFrame(canvas);

  // analogWrite(MotorPWM, 255/2);
  
  delay(1000);
}
