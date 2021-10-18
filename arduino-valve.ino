/* Date: 1 September 21
   Prototype Code
   Desc: - Control system with desired value
         - Pressure Range: 0 - 1.5 bar                    
         - Servo controlled by I2C module 
   by: Tutla Ayatullah
*/

#include<Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define USE_COUNT       0
#define USE_NUMBER      1 /*Pressure input method: (1) command input with number, (0) incremental input*/
#define USE_FILTER_5HZ  0 /*set 0 to use 10Hz filter*/

#define tab   Serial.print("\t")
#define enter Serial.println()

#define servo_freq    50     /*Servo Pulse Frequency */
#define max_pressure  1.5    /*Max Pressure*/
#define min_pressure  0.2      /*Min Pressure*/
#define max_pulse     440    /*Max Servo Pulse Width*/
#define min_pulse     100    /*Min Servo Pulse Width*/
#define max_angle     180    /*Max Servo Angle*/
#define min_angle     0      /*Min Servo Angle*/
#define en_servo_pin  4      /*Enable Servo*/
#define motor_pwm     255    /*Compressor MOSFET PWM*/
#define scale         1      /*Serial Plotter Scale*/
#define enable_servo  digitalWrite(en_servo_pin,0)
#define disable_servo digitalWrite(en_servo_pin,1)
const bool    start_chamber[3]= {1,0,0};
const float   dt = 0.01;

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
      y_out[0]  = -(a[0] * y_out[1] + a[1] * y_out[2] + a[2] * y_out[3]);
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
#if  USE_COUNT
  bool start_data = 0;
  const uint8_t dt_ms = 10; /*Sampling Time in ms*/
  const uint8_t T = 10;
  const uint16_t n = (1000 / dt) * T;
  unsigned int count = 0;
  uint32_t t1 = 0, t2 = 0, t = 0,t3=0,t4=0;
#endif

/*PID*/                                   /*3*/   /*2*/    /*1*/
const float Kp[3] = {20,25,20};     /*57*/ /*230*/  /*250*/
const float Ki[3] = {150,150 ,150}; /*2.78*/ /*3.75*/ /*3.75*/
const float Kd[3] = {0,0,0};       /*0.5*/ /*350*/  /*400*/
float   set_point[3] = {min_pressure,min_pressure,min_pressure};
float   proportional[3] = {0,0,0};
float   I[3] = {0,0,0}; 
float   derivative[3] = {0,0,0};
float   err[3]      = {0,0,0}, err_f[3] ={0,0,0};
float   pre_err[3]  = {0,0,0};
float   sig[3]      = {0,0,0};
uint8_t pos[3]      = {180,180,180};
uint8_t pos_inv[3]  = {0,0,0};

/*Sensor*/
uint16_t adc[3] = {0,0,0};
uint8_t  MPX[3] = {0,1,2}; /*Sensor Port*/
float    P[3]   = {0,0,0};
float    Pf[3]  = {0,0,0};

/*Compressor*/
bool    motorOn   = 0;
uint8_t motor[3]  = {9,6,5}; /*Motor Port*/

/*Servo Port*/
uint8_t servo[3] = {4,8,12};

/*Communication*/
bool flag = 0;

uint16_t d2p(uint8_t deg) {
  uint16_t pulse;
  pulse = map(deg, 0, 180, min_pulse, max_pulse);
  return pulse;
}

void FilterInit(){
/*Filter*/

#if USE_FILTER_5HZ
  /*Filter coefficient with wc = 5 Hz*/
  const float b[4] = {2.8982*pow(10,-3), 8.6946*pow(10,-3), 8.6946*pow(10,-3),  2.8982*pow(10,-3)};
  const float a[3] = {-2.37409, 1.92936, -0.53208};
#else
  /*Filter coefficient with wc = 10 Hz*/
  const float b[4] = {0.018099, 0.054297, 0.054297, 0.018099};
  const float a[3] = {-1.76004, 1.18289, -0.27806};

  const float b_f[4] = {0.0004, 0.0012, 0.0012,  0.0004}; /*2.5 Hz*/
  const float a_f[3] = {-2.6862, 2.4197, -0.7302};

  const float b_1[4] = {0.2915*pow(10,-4), 0.8744*pow(10,-4), 0.8744*pow(10,-4),  0.2915*pow(10,-4)}; /*1 Hz*/
  const float a_1[3] = {-2.8744, 2.7565, -0.8819};
  
    /*Filter coefficient with wc = 5 Hz*/
  const float b_5[4] = {2.8982*pow(10,-3), 8.6946*pow(10,-3), 8.6946*pow(10,-3),  2.8982*pow(10,-3)};
  const float a_5[3] = {-2.37409, 1.92936, -0.53208};
#endif

  for (int i =0;i<3;i++){
    input[i].set_coefficient(a_f,b_f);
    output[i].set_coefficient(a_5,b_5);
  }
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
  char incByte = 0;
  const float inc = 0.01;
  
  if (Serial.available() > 0) {
    if(USE_NUMBER){
      incByte = Serial.read();
      if(incByte == 'a'){
        motorOn = 1;
        I[0] = 0; I[1] = 0; I[2] = 0;
        for(int i=0;i<3;i++){
          if(start_chamber[i]){
            set_point[i] = min_pressure;
            analogWrite(motor[i],motor_pwm);
            enable_servo;
          }
        }
      }
      else if(incByte == 's'){
        motorOn = 0;
        I[0] = 0; I[1] = 0; I[2] = 0;
        enable_servo;
        for(int i=0;i<3;i++){
          analogWrite(motor[i],0);
          set_point [i] = min_pressure;
          pwm1.setPWM(servo[i], 0, d2p(180));  
        }
      }
      else if((incByte == 'p') && motorOn){
        for(int i=0;i<3;i++){
          if(start_chamber[i]){
            set_point[i] = Serial.parseFloat();
            if (set_point[i] > max_pressure) set_point[i] = max_pressure;
            if (set_point[i] < min_pressure) set_point[i] = min_pressure;
          }  
        }
      }
    }
    else{
      incByte = Serial.read();
      if(incByte == 'a'){
        motorOn = 1;
        I[0] = 0; I[1] = 0; I[2] = 0;
        for(int i=0;i<3;i++){
          if(start_chamber[i]){
            analogWrite(motor[i],motor_pwm);
            enable_servo;
          }
        }
      }
      else if(incByte == 's'){
        motorOn = 0;
        I[0] = 0; I[1] = 0; I[2] = 0;
        enable_servo;
        for(int i=0;i<3;i++){
          analogWrite(motor[i],0);
          set_point [i] = min_pressure;
          pwm1.setPWM(servo[i], 0, d2p(180));  
        }
      }
      else{
        if(motorOn){
          switch(incByte){
            case 'c': set_point[0] += inc; break;
            case 'd': set_point[0] -= inc; break;          
            case 'g': set_point[1] += inc; break;
            case 'h': set_point[1] -= inc; break;
            case 'y': set_point[2] += inc; break;
            case 'x': set_point[2] -= inc; break;                                             
          }         
        }
        
        /*set point limit*/
        for (int i=0;i<3;i++){
          if(set_point[i] > max_pressure) set_point[i] = max_pressure;
          else if(set_point[i] < min_pressure) set_point[i] = min_pressure;
        }
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

  TimerInit();
  FilterInit();
  
  for(int i = 0;i<3;i++){
    if(start_chamber[i]){
      pwm1.setPWM(servo[i], 0, d2p(180));
          
      /*Sensor*/  
      pinMode(MPX[i], INPUT);
      pinMode(motor[i], OUTPUT);
    
      /*Compressor*/
      analogWrite(motor[i],0);
    }
  }
    
  pinMode(en_servo_pin,OUTPUT);
  enable_servo;
  delay(500);
}

void loop() {
  read_input();

  /*Print Data*/
  
  if(flag){
    for(int i=0;i<3;i++){
      if(start_chamber[i]){
        Serial.print(set_point[i]*scale); tab;
        Serial.print(Pf[i]*scale); tab;
      }
    }
    Serial.print(max_pressure*scale); tab;
    Serial.print(min_pressure*scale);
    enter;
 }
}

ISR(TIMER1_COMPA_vect){
  sei();
  for(int i=0;i<3;i++){
    if(start_chamber[i]){
      /*Read*/
      adc[i] = analogRead(MPX[i]);
      //P[i] = 5.28*(adc[i]/1000.0)-0.175;
      P[i] = 5.28*(adc[i]/1000.0)-0.125;
      //Pf[i]=P[i];
      Pf[i] = input[i].filter(P[i]);
  
      /*Calculate PI Control*/
      err[i] = (set_point[i] - Pf[i]);
      proportional[i] = err[i];
      I[i] = I[i] + err[i]*dt;
      derivative[i] = (err[i]-pre_err[i]);
      sig[i] = (Kp[i]*proportional[i])+(Ki[i]*I[i])+(Kd[i]*derivative[i]);
      pos[i] = output[i].filter(sig[i]);
      pos_inv[i] = map(pos[i],0,180,180,0);
      pre_err[i] = err[i];
  
      /*Saturation*/
      if(pos_inv[i] > max_angle) {
        pos_inv[i] = max_angle;
        I[i] -= err[i];
      }
      else if (pos_inv[i] < min_angle) {
        pos_inv[i] = min_angle;
        I[i] -= err[i];
      }
      
      /*Execute*/      
      if(motorOn)pwm1.setPWM(servo[i], 0, d2p(pos_inv[i]));    
    }
  }
  flag = 1;
}
