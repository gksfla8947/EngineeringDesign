#include<Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0
#define SERVO_SPEED 200
#define INTERVAL 20

#define _DUTY_MAX 2000
#define _DUTY_NEU 1600
#define _DUTY_MIN 1000

float last_sampling_time;
float duty_chg_per_interval;
int duty_target, duty_curr;
float dist_raw, dist_ema;

Servo myservo;

void setup() {
  duty_chg_per_interval = (int)((float)(_DUTY_MAX - _DUTY_MIN) * ((float)SERVO_SPEED / (float)180) * ((float)INTERVAL / (float)1000));
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);
  dist_raw, dist_ema = 0;
  last_sampling_time = 0;

  Serial.begin(57600);
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;

  dist_raw = ir_distance() + 30;
  dist_ema = 0.1 * dist_raw + 0.9 * dist_ema;

  Serial.println(dist_ema);
  if(dist_ema < 255) {
    duty_target = _DUTY_NEU + 200;
    if(duty_target >= duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_target < duty_curr) {
        duty_curr = duty_target;
      }
    }
  }
  else {
    duty_target = _DUTY_NEU - 200;
    if(duty_target <= duty_curr) {
      duty_curr -= duty_chg_per_interval;
      if(duty_target > duty_curr) {
        duty_curr = duty_target; 
      }
    }
  }

  myservo.writeMicroseconds(duty_curr);
  
  last_sampling_time += INTERVAL;
}

float ir_distance() {
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
