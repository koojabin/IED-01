#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10                    //[3030]SERVO를 10번 핀에 연결
#define PIN_IR A0     //[3037]적외선 센서를 A0에 연결

// Framework setting
#define _DIST_TARGET 255            // [3040] 레일플레이트의 중간지점(목표지점=25.5cm)
#define _DIST_MIN 100                    //[3035] 측정 최소 거리
#define _DIST_MAX 450   //[3036] 측정 가능한 최대 거리

// Distance sensor
#define _DIST_ALPHA 0.1   //[3023] EMA 가중치

// Servo range 
#define _DUTY_MIN 1200                 //[3028] 서보 각도 최소값
#define _DUTY_NEU 1400    //[3038] 레일 수평 서보 펄스폭
#define _DUTY_MAX 1600    //[3031] 서보 최대값

// Servo speed control
#define _SERVO_ANGLE 30   //[3030] servo angle limit 실제 서보의 동작크기
          //[3023] 서보모터의 작동 범위(단위 : degree)
#define _SERVO_SPEED 100            // [3040] 서보의 각속도(초당 각도 변화량)

// Event periods
#define _INTERVAL_DIST 20   //[3039]적외선 센서 측정 간격
#define _INTERVAL_SERVO 20         //[3046]서보갱신간격
#define _INTERVAL_SERIAL 100       //[3030]시리얼 플로터 갱신간격

// PID parameters
#define _KP 1.7     //[3039] 비례 제어의 상수 값
#define _KD 90
#define _KI 0.03

#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.


// Servo instance     //[3046]서보간격
Servo myservo;

float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.
// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;     //[3034]적외선센서로 측정한 거리값과 ema필터를 적용한 거리값
float dist_min,dist_max;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;   //[3039] interval간격으로 동작 시행을 위한 정수 값
bool event_dist, event_servo, event_serial; //[3023] 적외선센서의 거리측정, 서보모터, 시리얼의 이벤트 발생 여부

// Servo speed control
int duty_chg_per_interval;  //[3039] interval 당 servo의 돌아가는 최대 정도
int duty_target, duty_curr;      //[3030]servo 목표 위치, servo 현재 위치
int duty_neutral = 1320;

int a=75,b=360;
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 


void setup() {
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED,OUTPUT);           //[3030]LED를 연결[3027]
myservo.attach(PIN_SERVO);  //[3039]servo를 연결

// initialize global variables
dist_min = _DIST_MIN; //[3030] 측정값의 최소값
dist_max= _DIST_MAX; //[3032] 측정값의 최대값
dist_target = _DIST_TARGET; //[3023] 목표 거리 위치

// move servo to neutral position
myservo.writeMicroseconds(duty_neutral);//[3030]서보를 레일이 수평이 되는 값으로 초기화

// initialize serial port
Serial.begin(57600);  //[3039] 시리얼 모니터 속도 지정 

// convert angle speed into duty change per interval.
  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
  error_prev = 0; 
}
  

void loop() 
{
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) 
  {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) 
  {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) 
  {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }
  if(event_dist) 
  {
    event_dist = false;    
    dist_ema = filtered_ir_distance();  
    error_curr = dist_target - dist_ema; 
    pterm =  _KP*error_curr;
    dterm = _KD*(error_curr - error_prev);
    iterm = _KI * error_curr;
    control = dterm + pterm + iterm;
    duty_target = _DUTY_NEU + control;
    if (duty_target > _DUTY_MAX) 
    {
      duty_target = _DUTY_MAX;
    }
    else if (duty_target < _DUTY_MIN) 
    {
      duty_target = _DUTY_MIN;
    } 
    error_prev = error_curr;
  }
  if(event_servo) 
  {
    event_servo = false;
    if(duty_target > duty_curr) 
    {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) 
      {
        duty_curr = duty_target;
      }
    }
    else 
    {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) 
      {
        duty_curr = duty_target;
      }    
    }
    myservo.writeMicroseconds(duty_curr);//[3034]
  }
  if(event_serial) 
  {
    event_serial = false;               // [3030]
    Serial.print("IR:");
    Serial.print(dist_ema);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
// ================
float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}

//float ir_distance(void){ // return value unit: mm
//  float val; //[3031] 
//  float volt = float(analogRead(PIN_IR));
//  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
//  float dist_cali = 100 + 300.0 / (b - a) * (val - a);
//  dist_ema = (_DIST_ALPHA * dist_cali) + (1 - _DIST_ALPHA) * dist_ema;
//  return dist_ema;
//}
//
//float ir_distance_filtered(void){ // return value unit: mm
//  return ir_distance(); // for now, just use ir_distance() without noise filter.
//}
