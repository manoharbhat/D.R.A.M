#include <Wire.h>
#include <Encoder.h>
#include <PID_v1.h>
#include "WindowedFilter.hpp"
const int l_f_en = 2;
const int l_f_p = 22;
const int l_f_n = 23;
const int l_b_en = 3;
const int l_b_p = 27;
const int l_b_n = 26;
const int r_f_en = 4;
const int r_f_p = 30;
const int r_f_n = 31;
const int r_b_en = 5;
const int r_b_p = 35;
const int r_b_n = 34;
const int battery_pin = A8;


 double set_speed[]={0.0,0.0,0.0,0.0}; // lf,rf,lb,rb
 double cur_speed[]={0.0,0.0,0.0,0.0};
double pid_out[]={0.0,0.0,0.0,0.0};
double pwm[]={0,0,0,0};

 
  double previous_position[] = {-999.0,-999.0,-999.0,-999.0};
  unsigned long previous_speed_time[] ={millis(),millis(),millis(),millis()};


  static constexpr double STOPPED_PWM{0.0};
  static constexpr double MAX_PWM{255.0};
  static constexpr double MIN_PWM{0.0};
int motors=4;
const double battery_factor = 7.2 / 1024;


Encoder l_f_enc(A0, A1);
Encoder l_b_enc(A2, A3);
Encoder r_f_enc(A4, A5);
Encoder r_b_enc(A6, A7);


PID lfPID(&cur_speed[0], &pid_out[0], &set_speed[0],2,5,1, REVERSE);
PID rfPID(&cur_speed[1], &pid_out[1], &set_speed[1],2,5,1, REVERSE);
PID lbPID(&cur_speed[2], &pid_out[2], &set_speed[2],2,5,1, REVERSE);
PID rbPID(&cur_speed[3], &pid_out[3], &set_speed[3],2,5,1, REVERSE);


void setup() {
  Wire.begin();
    pinMode(l_f_en, OUTPUT);
    pinMode(l_f_p, OUTPUT);
    pinMode(l_f_n, OUTPUT);
    pinMode(l_b_en, OUTPUT);
    pinMode(l_b_p, OUTPUT);
    pinMode(l_b_n, OUTPUT);
    pinMode(r_f_en, OUTPUT);
    pinMode(r_f_p, OUTPUT);
    pinMode(r_f_n, OUTPUT);
    pinMode(r_b_en, OUTPUT);
    pinMode(r_b_p, OUTPUT);
    pinMode(r_b_n, OUTPUT);
    pinMode(battery_pin, INPUT);
 lfPID.SetOutputLimits(0.0, 255.0);
  rfPID.SetOutputLimits(0.0, 255.0);

 lbPID.SetOutputLimits(0.0, 255.0);
 rbPID.SetOutputLimits(0.0, 255.0);

    lfPID.SetMode(MANUAL);
        rfPID.SetMode(MANUAL);
            lbPID.SetMode(MANUAL);
                rbPID.SetMode(MANUAL);
  
  Serial.begin(115200);
}


void loop() {
  if(Serial.available())
  {
    parseCommand();
    sendReply();
    delay(10);
  }



    updateSpeed(0, l_f_enc.read());

     updateSpeed(1, r_f_enc.read());
  
     updateSpeed(2, l_b_enc.read());

      updateSpeed(3, r_b_enc.read());


updatePwm(0);
updatePwm(1);
updatePwm(2);
updatePwm(3);

lfMotor();
rfMotor();
lbMotor();
rbMotor();

setPID();

  

}


void parseCommand()
{
  while(Serial.read() != '$');
        set_speed[0] = Serial.parseFloat();
        set_speed[1] = Serial.parseFloat();
          set_speed[2] = Serial.parseFloat();
            set_speed[3] = Serial.parseFloat();
        

 
 
    
  Serial.read(); // Take the newline out of the receive buffer
}
void sendReply()
{
  Serial.print("$");
  Serial.print(previous_position[0]);
  Serial.print(",");
  Serial.print(previous_position[1]);
  Serial.print(",");
  Serial.print(previous_position[2]);
  Serial.print(",");
  Serial.print(previous_position[3]);
  Serial.print(",");
  Serial.print(cur_speed[0]);
  Serial.print(",");
  Serial.print(cur_speed[1]);
  Serial.print(",");
  Serial.print(cur_speed[2]);
  Serial.print(",");
  Serial.print(cur_speed[3]);
  Serial.print(",");
  Serial.print(battery_factor * analogRead(battery_pin));
 
// Serial.print(set_speed[0]);
// Serial.print(",");
// 
// Serial.print(set_speed[1]);
// Serial.print(",");
// 
// Serial.print(set_speed[2]);
//  Serial.print(",");
//
//  
//    Serial.print(set_speed[3]);

// 
// Serial.print(pwm[0]);
// Serial.print(",");
// 
// Serial.print(pwm[1]);
// Serial.print(",");
// 
// Serial.print(pwm[2]);
//  Serial.print(",");
//
//  
//    Serial.print(pwm[3]);

  Serial.println();
}


  void updateSpeed(int i,int curr_pos)
  {

    unsigned long current_time = micros();
    double current_position = curr_pos;
    
    double delta_pos = current_position - previous_position[i];
    if(current_time-previous_speed_time[i] > 10)
    {
      double delta_t_secs = (current_time - previous_speed_time[i])/1000000.0;
      cur_speed[i] = (delta_pos / delta_t_secs);
    }
    previous_position[i] = current_position;
    previous_speed_time[i] = current_time;

  }

  
  
  void updatePwm(int m){

    if(set_speed[m] < 0.01 && set_speed[m] > -0.01)
    {
      pwm[m] = STOPPED_PWM;
    }
    else
    {
 
     lfPID.Compute();

      rfPID.Compute();

      lbPID.Compute();

       rbPID.Compute();
  pwm[m] += pid_out[m];
      pwm[m] = constrain(pwm[m], MIN_PWM, MAX_PWM);
    }
  }

  void lfMotor(){


      if (set_speed[0] > 0){
              analogWrite(l_f_en,pwm[0]);
              digitalWrite(l_f_p,HIGH);
              digitalWrite(l_f_n,LOW);
      
      }
      else{
        
              analogWrite(l_f_en,pwm[0]);
                 digitalWrite(l_f_p,LOW);
              digitalWrite(l_f_n,HIGH);
      
  }}
  void rfMotor(){
 
       if (set_speed[1] > 0){
              analogWrite(r_f_en,pwm[1]);
              digitalWrite(r_f_p,HIGH);
              digitalWrite(r_f_n,LOW);
      
      }
      else{
        
              analogWrite(r_f_en,pwm[1]);
        digitalWrite(r_f_p,LOW);
              digitalWrite(r_f_n,HIGH);
      
  }}
  void lbMotor(){

       if (set_speed[2] > 0){
              analogWrite(l_b_en,pwm[2]);
              digitalWrite(l_b_p,HIGH);
              digitalWrite(l_b_n,LOW);
      
      }
      else{
        
              analogWrite(l_b_en,pwm[2]);
          digitalWrite(l_b_p,LOW);
              digitalWrite(l_b_n,HIGH);
      
  }}
   void rbMotor(){
  
        if (set_speed[3] > 0){
              analogWrite(r_b_en,pwm[3]);
              digitalWrite(r_b_p,HIGH);
              digitalWrite(r_b_n,LOW);
      
      }
      else{
        
              analogWrite(r_b_en,pwm[3]);
              digitalWrite(r_b_p,LOW);
              digitalWrite(r_b_n,HIGH);
      
  }
   
    
    
    }


 void setPID(){
  
  if(set_speed[0] < 0.01 && set_speed[0] > -0.01)
    {
      lfPID.SetMode(MANUAL);
    }
    else
    {
     lfPID.SetMode(AUTOMATIC);
    }
if(set_speed[1] < 0.01 && set_speed[1] > -0.01)
    {
      rfPID.SetMode(MANUAL);
    }
    else
    {
     rfPID.SetMode(AUTOMATIC);
    }
  if(set_speed[2] < 0.01 && set_speed[2] > -0.01)
    {
      lbPID.SetMode(MANUAL);
    }
    else
    {
     lbPID.SetMode(AUTOMATIC);
    }
  
  if(set_speed[3] < 0.01 && set_speed[3] > -0.01)
    {
      rbPID.SetMode(MANUAL);
    }
    else
    {
     rbPID.SetMode(AUTOMATIC);
    }
  
  }
