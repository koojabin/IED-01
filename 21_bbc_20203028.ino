// Arduino pin assignment
#include <Servo.h>
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define _DIST_ALPHA 0.1

#define _SERVO_SPEED 50

#define _DUTY_MIN 440
#define _DUTY_NEU 1420
#define _DUTY_MAX 2400

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

#define INTERVAL 20

Servo myservo;
unsigned long last_sampling_time; // unit: ms
int a, b; // unit: mm
float dist_ema=0;
float duty_chg_per_interval;
int toggle_interval, toggle_interval_cnt;
float duty_target = 1320, duty_curr = 1320;
float pause_time;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1320);

  duty_target = duty_curr = _POS_START;

  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * (float)(INTERVAL) / 1000;
// initialize serial port
// initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
  
  Serial.begin(57600);
  
  a = 71;
  b = 355;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  dist_ema = (_DIST_ALPHA * dist_cali) + (1 - _DIST_ALPHA) * dist_ema;
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_ema);
  Serial.print("duty_chg_per_interval:");
  Serial.println(duty_chg_per_interval);
  if (dist_cali>255)
  {
    duty_target = 800;
    if(duty_target > duty_curr) 
    {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else 
    {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    //myservo.writeMicroseconds(800);
  }
  else if(dist_cali<255)
  {
    duty_target = 1800;
    if(duty_target > duty_curr) 
    {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else 
    {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    //myservo.writeMicroseconds(1800);
  }
  // toggle duty_target between _DUTY_MIN and _DUTY_MAX.
  if(toggle_interval_cnt >= toggle_interval) {
    toggle_interval_cnt = 0;
    if(duty_target == _POS_START) duty_target = _POS_END;
    else duty_target = _POS_START;
  }
  else {
    toggle_interval_cnt++;
  }
  myservo.writeMicroseconds(duty_curr);
// update last sampling time
  last_sampling_time += INTERVAL;
  delay(20);
}
