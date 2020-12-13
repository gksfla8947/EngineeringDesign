#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9         
#define PIN_SERVO 10    
#define PIN_IR A0   

// Framework setting
#define _DIST_TARGET 255  
#define _DIST_MIN 100   
#define _DIST_MAX 410              

// Distance sensor
#define _DIST_ALPHA 0.5  
#define a 64
#define b 298

// Servo range
#define _DUTY_MIN 1570
#define _DUTY_NEU 1670 
#define _DUTY_MAX 1770           

// Servo speed control
#define _SERVO_ANGLE 30  
#define _SERVO_SPEED 100

// Event periods
#define _INTERVAL_DIST 20   
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 0.7
#define _KI 0.0   
#define _KD 100.0

// ir filter
#define LENGTH 70
#define k_LENGTH 15


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; 
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,last_sampling_time_serial;


bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr, duty_neutral; 

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 


void setup() {
// initialize GPIO pins for LED and attach servo 
    pinMode(PIN_LED, OUTPUT);
    myservo.attach(PIN_SERVO); 

    // initialize global variables
    duty_curr = _DUTY_MIN; 
    duty_target = _DIST_TARGET;
    duty_neutral = _DUTY_NEU;
    last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0;
 
    dist_raw = dist_ema = 0;
    pterm = iterm = dterm = 0; 

    error_prev = 0;

    // move servo to neutral position
    myservo.writeMicroseconds(_DUTY_NEU); 
    //while(1) {}

    // initialize serial port
    Serial.begin(57600); 

    // convert angle speed into duty change per interval.
    duty_chg_per_interval = int((_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED /  _SERVO_ANGLE) * (float(_INTERVAL_DIST) / 1000.0));

}
  
void loop() {
    /////////////////////
    // Event generator //
    /////////////////////
    unsigned long time_curr = millis(); 

    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    ////////////////////
    // Event handlers //
    ////////////////////

    if(event_dist) {
        event_dist = false;
        // get a distance reading from the distance sensor
        dist_raw = ir_distance_filtered(); 
        dist_ema = _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema; 


        // PID control logic
        error_curr = _DIST_TARGET - dist_raw; 
        
        pterm = _KP * error_curr;
        dterm = _KD * (error_curr - error_prev);
        control = pterm + dterm;
        duty_target = _DUTY_NEU + control;

        // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
        if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
        if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

        // update error_prev
        error_prev = error_curr;
    }
  
    if(event_servo) {
        event_servo = false;
        // adjust duty_curr toward duty_target by duty_chg_per_interval
        if(duty_target > duty_curr) { 
            duty_curr += duty_chg_per_interval;
            if(duty_curr > duty_target) duty_curr = duty_target;
        }
        else {
            duty_curr -= duty_chg_per_interval;
            if(duty_curr < duty_target) duty_curr = duty_target;
        }

        // update servo position
        myservo.writeMicroseconds((int)duty_curr); 

        event_servo = false; 
    }
  
    if(event_serial) {
        event_serial = false;
        Serial.print("dist_ir:");
        Serial.print(dist_raw);
        Serial.print(",pterm:");
        Serial.print(map(pterm,-1000,1000,510,610));
        Serial.print(",dterm:");
        Serial.print(map(dterm,-1000,1000,510,610));
        Serial.print(",duty_target:");
        Serial.print(map(duty_target,1000,2000,410,510));
        Serial.print(",duty_curr:");
        Serial.print(map(duty_curr,1000,2000,410,510));
        Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    }
}

float ir_distance(void){ // return value unit: mm
    float val; 
    float volt = float(analogRead(PIN_IR)); 
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0; 
    return val; 
}

float ir_distance_filtered(void){ // return value unit: mm
    int cnt = 0;
    int sum = 0;
    int dist_list[LENGTH+1];
    while (cnt < LENGTH)
    {
        dist_list[cnt] = 100 + 300.0 / (b - a) * (ir_distance() - a);
        sum += dist_list[cnt];
        cnt++;
    }

    for (int i = 0; i < LENGTH-1; i++){
        for (int j = i+1; j < LENGTH; j++){
            if (dist_list[i] > dist_list[j]) {
                float tmp = dist_list[i];
                dist_list[i] = dist_list[j];
                dist_list[j] = tmp;
            }
        }
    }
    
    for (int i = 0; i < k_LENGTH; i++) {
        sum -= dist_list[i];
    }
    for (int i = 1; i <= k_LENGTH; i++) {
        sum -= dist_list[LENGTH-i];
    }

    return sum/(LENGTH-2*k_LENGTH);
}
