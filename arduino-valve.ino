/* Date: 1 August 21
   Prototype Code
   Desc: - Control system with desired value
         - Pressure Range: 0.2 - 0.7 bar                    
         - Servo controlled by I2C module 
   by: Tutla Ayatullah
*/

#include<Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define tab   Serial.print("\t")
#define enter Serial.println()

#define servo_freq    50    /*Servo Pulse Frequency */
#define max_pressure  0.7   /*Max Pressure*/
#define min_pressure  0.2   /*Min Pressure*/
#define max_pulse     440   /*Max Servo Pulse Width*/
#define min_pulse     100   /*Min Servo Pulse Width*/
#define max_angle     180   /*Max Servo Angle*/
#define min_angle     0     /*Min Servo Angle*/
#define en_servo_pin  4     /*Enable Servo*/
#define motor_pwm     255   /*Compressor MOSFET PWM*/
#define scale         10    /*Serial Plotter Scale*/
#define enable_servo  digitalWrite(en_servo_pin,0)
#define disable_servo digitalWrite(en_servo_pin,1)

class Butterworth {
  /* 3rd order low-pass butterworth filter */
  public:
    float a[3], b[4];
    float y_out[4] = {0.0000, 0.0000, 0.0000, 0.0000};
    float y_in[4]  = {0.0000, 0.0000, 0.0000, 0.0000};

    void set_coefficient(const float co_a[3], const float co_b[4]) {
      for (int i = 0; i < 3; i++) {
        a[i] = co_a[i];
      }
      for (int i = 0; i < 4; i++) {
        b[i] = co_b[i];
      }
    }
    float filter(float y) {
      y_in[0]  = y;
      y_out[0]  = (a[0] * y_out[1] + a[1] * y_out[2] + a[2] * y_out[3]);
      y_out[0] += (b[0] * y_in[0]  + b[1] * y_in[1]  + b[2] * y_in[2] + b[3] * y_in[3]);

      /*Index Shifting*/
      y_out[3] = y_out[2];
      y_out[2] = y_out[1];
      y_out[1] = y_out[0];

      y_in[3]  = y_in[2];
      y_in[2]  = y_in[1];
      y_in[1]  = y_in[0];

      return y_out[0];
    }
};

Butterworth input[3],output[3];
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

/*Global Variable*/
/*Timer*/
const uint8_t dt = 10; /*Sampling Time in ms*/
const uint8_t T = 10;
const uint16_t n = (1000 / dt) * T;
unsigned int count = 0;
uint32_t t1 = 0, t2 = 0, t = 0,t3=0,t4=0;

/*PID*/                                 /*3*/   /*2*/    /*1*/
const float Kp[3] = {230,230,230};     /*230*/ /*230*/  /*250*/
const float Ki[3] = {4.0,4.0,4.0};    /*4.0*/ /*3.75*/ /*3.75*/
const float Kd[3] = {375,375,375};   /*375*/ /*350*/  /*400*/
static float I[3] = {0,0,0}; 
float   set_point[3] = {min_pressure,min_pressure,min_pressure};
float   err[3]    = {0,0,0};
float   pre_err[3]= {0,0,0};
float   sig[3]    = {0,0,0};
uint8_t pos[3]    = {0,0,0};
const bool    start_chamber[3]= {1,1,1};

/*Sensor*/
uint16_t adc[3] = {0,0,0};
uint8_t  MPX[3] = {0,1,2}; /*Sensor Port*/
float    P[3]   = {0,0,0};
float    Pf[3]  = {0,0,0};

/*Compressor*/
bool    motor_status = 0;
uint8_t motor[3]     = {9,6,5}; /*Motor Port*/

/*Servo Port*/
uint8_t servo[3] = {4,8,12};

/*Communication*/
char incByte = 0;
bool get_data = 0,flag = 0;
const float inc = 0.02;

/*Filter coefficient with wc = 5 Hz*/
const float b_1[4] = {2.8982*pow(10,-3), 8.6946*pow(10,-3), 8.6946*pow(10,-3),  2.8982*pow(10,-3)};
const float a_1[3] = {2.37409, -1.92936, 0.53208};

/*Filter coefficient with wc = 10 Hz*/
const float b_2[4] = {0.018099, 0.054297, 0.054297, 0.018099};
const float a_2[3] = {1.76004, -1.18289, 0.27806};

uint16_t d2p(uint8_t deg) {
  uint16_t pulse;
  pulse = map(deg, 0, 180, min_pulse, max_pulse);
  return pulse;
}

void TimerInit(){
  /*Initialize timer1 with Sampling Frequency = 100 Hz*/
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 2499;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void read_input(){
  if (Serial.available() > 0) {
    incByte = Serial.read();
    if(incByte == 'a'){
        motor_status = 1;
        I[0] = 0; I[1] = 0; I[2] = 0;
    }
    else if(incByte == 's'){
        motor_status = 0;
        I[0] = 0; I[1] = 0; I[2] = 0;
    }
    else{
      if(motor_status){
        if(incByte == 'c')      set_point[0] += inc;
        else if(incByte == 'd') set_point[0] -= inc;
        else if(incByte == 'g') set_point[1] += inc;
        else if(incByte == 'h') set_point[1] -= inc;
        else if(incByte == 'y') set_point[2] += inc;
        else if(incByte == 'x') set_point[2] -= inc;          
      }
    }
  }
}

void setup() {
  /*Serial Communication*/
  Serial.begin(115200);
  
  /*PWM Initialization*/
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(servo_freq);
  
  for(int i = 0;i<3;i++){
    if(start_chamber[i]){
      pwm1.setPWM(servo[i], 0, d2p(pos[i]));
    
      /*Filter*/
      input[i].set_coefficient(a_1, b_1);
      output[i].set_coefficient(a_1,b_1);
    
      /*Sensor*/  
      pinMode(MPX[i], INPUT);
      pinMode(motor[i], OUTPUT);
    
      /*Compressor*/
      analogWrite(motor[i],0);
    }
  }
  TimerInit();
  pinMode(en_servo_pin,OUTPUT);
  disable_servo;
  delay(500);
}

void loop() {
  read_input();
  
  for(int i = 0;i<3;i++){
    if(motor_status && start_chamber[i]){
      analogWrite(motor[i],motor_pwm);
      enable_servo;
    }
    else{
      analogWrite(motor[i],0);
      set_point [i] = min_pressure;  
    } 
  }

  if(flag){
    for(int i=0;i<3;i++){
      Serial.print(set_point[i]*scale);  tab;
    }
    enter; 
    flag = 0;
  }
}

ISR(TIMER1_COMPA_vect){
  sei();
  for(int i=0;i<3;i++){
    if(start_chamber[i]){
      /*Read*/
      adc[i] = analogRead(MPX[i]);
      P[i] = 5.7*(adc[i]/1000.0)-0.1926; /*(bar)*/
      Pf[i] = input[i].filter(P[i]);
  
      /*Calculate PI Control*/
      err[i] = (set_point[i] - Pf[i]);
      I[i] += err[i];
      sig[i] = (Kp[i]*err[i])+(Ki[i]*I[i])+(Kd[i]*(err[i]-pre_err[i]));
      pos[i] = output[i].filter(sig[i]);
      pre_err[i] = err[i];
  
      /*Saturation*/
      if (pos[i] > max_angle) {
         pos[i] = 180;
        I[i] -= err[i];
      }
      else if (pos[i] < min_angle) {
         pos[i] = 0;
        I[i] -= err[i];
      }  
    }
   }
    
   /*Execute*/
   for(int i=0;i<3;i++){
    if(start_chamber[i]){
        pwm1.setPWM(servo[i], 0, d2p(pos[i]));  
    }   
   }
  flag = 1;
}
