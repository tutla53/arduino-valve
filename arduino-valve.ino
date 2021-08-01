/* Date: 24  Nov 2020
   Prototype Code
   Desc: - Control system with desired value
         - Pressure Range: 0.1 - 0.35 bar                    
         - Servo controlled by I2C module 
   by: Tutla Ayatullah
*/

#include<Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define tab   Serial.print("\t")
#define enter Serial.println()

#define servo_freq    50   /*Servo Pulse Frequency */
#define max_pressure  0.80   /*Max Pressure*/
#define min_pressure  0.20   /*Min Pressure*/
#define max_pulse     450  /*Max Servo Pulse Width*/
#define min_pulse     100  /*Min Servo Pulse Width*/
#define max_angle     180  /*Max Servo Angle*/
#define min_angle     0    /*Min Servo Angle*/
#define en_servo_pin  4    /*Enable Servo*/
#define motor_pwm     255  /*Compressor MOSFET PWM*/
#define scale         10    /*Serial Print Scale*/
#define enable_servo  digitalWrite(en_servo_pin,0)
#define disable_servo digitalWrite(en_servo_pin,1)

class Butterworth {
  /* 3rd order low-pass butterworth filter */
  public:
    float a[3], b[4];
    float y_out[4] = {0.0000, 0.0000, 0.0000, 0.0000};
    float y_in[4]  = {0.0000, 0.0000, 0.0000, 0.0000};

    void set_coefficient(float co_a[3], float co_b[4]) {
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
/*const uint8_t dt = 10; 
const uint8_t T = 10;
const uint16_t n = (1000 / dt) * T;
unsigned int count = 0;
uint32_t t1 = 0, t2 = 0, t = 0,t3=0,t4=0;
*/
/*PID*/                                  /*3*/   /*2*/    /*1*/
const float Kp[3] = {0.9,230,230};      /*230*/ /*230*/  /*250*/
const float Ki[3] = {0.04,4.0,4.0};    /*4.0*/ /*3.75*/ /*3.75*/
const float Kd[3] = {7,375,375};  /*375*/ /*350*/  /*400*/
float   I[3] = {0,0,0}; 
float   set_point[3] = {min_pressure,min_pressure,min_pressure};
float   err[3]    = {0,0,0};
float   pre_err[3]= {0,0,0};
float   sig[3]    = {0,0,0};
float   P_term[3] = {0,0,0};
float   I_term[3] = {0,0,0};
float   D_term[3] = {0,0,0};
uint8_t pos[3]    = {0,0,0};

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
String val = "";

/*Filter coefficient with wc = 2.5 Hz*/
float b_0[4] = {4.1655*pow(10,-4), 1.24964*pow(10,-3), 1.24964*pow(10,-3),  4.1655*pow(10,-4)};
float a_0[3] = {2.68616, -2.41966, 0.73017};

/*Filter coefficient with wc = 5 Hz*/
float b_1[4] = {2.8982*pow(10,-3), 8.6946*pow(10,-3), 8.6946*pow(10,-3),  2.8982*pow(10,-3)};
float a_1[3] = {2.37409, -1.92936, 0.53208};

/*Filter coefficient with wc = 10 Hz*/
float b_2[4] = {0.018099, 0.054297, 0.054297, 0.018099};
float a_2[3] = {1.76004, -1.18289, 0.27806};

uint16_t d2p(uint8_t deg) {
  uint16_t pulse;
  pulse = map(deg, 0, 180, min_pulse, max_pulse);
  return pulse;
}

void TimerInit(){
  /*Sampling Frequency = 100 Hz*/
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

void setup() {
  /*Serial Communication*/
  Serial.begin(115200);
  
  /*PWM Initialization*/
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(servo_freq);

  for(int i = 0;i<3;i++){
    pwm1.setPWM(servo[i], 0, d2p(pos[i]));
    
    /*Filter*/
    input[i].set_coefficient(a_0, b_0);
    output[i].set_coefficient(a_0,b_0);
    
    /*Sensor*/  
    pinMode(MPX[i], INPUT);
    pinMode(motor[i], OUTPUT);
    
    /*Compressor*/
    analogWrite(motor[i],0);
  }

  TimerInit();
  pinMode(en_servo_pin,OUTPUT);
  disable_servo;
  delay(500);
}

void loop() {
  
  if (Serial.available() > 0) {
    incByte = Serial.read();
    if (incByte != '\n') {
      if(isDigit(incByte)||(incByte=='.')){
        val += (char)incByte; 
      }
      else if(incByte == 'a'){
        motor_status = 1;
      }
      else if(incByte == 's'){
        motor_status = 0;
      }      
    }
    else {
      for(int i=0;i<3;i++){
        set_point[i] = val.toFloat();
        if (set_point[i] > max_pressure) set_point[i] = max_pressure;
        if (set_point[i] < min_pressure) set_point[i] = min_pressure;  
      }      
//      get_data = 1;
//      count = 0;
      val = "";
    }
  }
  
  if(motor_status){
    analogWrite(motor[0],motor_pwm);
    //analogWrite(motor[1],motor_pwm);
    //analogWrite(motor[2],motor_pwm);
    enable_servo;
  }
  else{
    for(int i=0;i<3;i++){
      analogWrite(motor[i],0);
      set_point [i] = min_pressure;  
    }
    disable_servo;
  }
  if(flag){
    Serial.print(set_point[0]*scale);  tab;
    //Serial.print(P_term[0]*scale);     tab;
    //Serial.print(I_term[0]*scale);     tab;
    //Serial.print(D_term[0]*scale);     tab;
    Serial.print(Pf[0]*scale);        tab;
    Serial.print(max_pressure*scale);  tab;
    Serial.print(min_pressure*scale);
    enter;   
    flag = 0;
  }   
}

ISR(TIMER1_COMPA_vect){
  sei();
  if(motor_status){
    for(int i=0;i<3;i++){
       /*Read*/
       adc[i] = analogRead(MPX[i]);
       P[i] = 0.0057*adc[i]-0.1926; /*(bar)*/
       Pf[i]  = input[i].filter(P[i]);
  
       /*Calculate PI Control*/
       err[i] = (set_point[i] - P[i]);
       I[i] += err[i];
       P_term[i] = Kp[i]*err[i];
       I_term[i] = Ki[i]*I[i];
       D_term[i] = Kd[i]*(err[i]-pre_err[i]);
       sig[i] = P_term[i] + I_term[i] + D_term[i];
       pos[i] = output[i].filter(sig[i]);
       //pos[i] = sig[i];
       pre_err[i] = err[i];
       
       /*Saturation*/
       if(pos[i] > max_angle) {
         pos[i] = 180;
         I[i] -= err[i];
       }
        else if(pos[i] < min_angle) {
         pos[i] = 0;
         I[i] -= err[i];
       }  
     } 
     /*Execute*/
     pwm1.setPWM(servo[0], 0, d2p(pos[0]));
     //pwm1.setPWM(servo[1], 0, d2p(pos[1]));
     //pwm1.setPWM(servo[2], 0, d2p(pos[2]));
     flag = 1;
  }
}
