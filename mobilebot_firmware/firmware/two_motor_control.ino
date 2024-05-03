#include<PID_v1.h>


#define enableA 9
#define enableB 11
#define in1 12
#define in2 13
#define in3 7
#define in4 8
#define Renco_Ac 3 //interrupt
#define Renco_Bc 5
#define Lenco_Ac 2 //interrupt
#define Lenco_Bc 4


unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_encoder_dir ="p";
String left_encoder_dir ="p";

double right_wheel_measured_vel = 0.0; 
double left_wheel_measured_vel = 0.0; 

bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_cmd_complete = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value [] ="00.00";
uint8_t value_idx = 0;  

double right_wheel_cmd_vel =0.0;
double left_wheel_cmd_vel =0.0;

unsigned long last_millis = 0;
const unsigned long intervel = 100;

double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;

double kp_r = 11.5;
double ki_r =7.5;
double kd_r =0.1;
double kp_l =12.8;
double ki_l =8.3;
double kd_l =0.1;

PID rightMotor(&right_wheel_measured_vel, &right_wheel_cmd, &right_wheel_cmd_vel, kp_r, ki_r, kd_r, DIRECT);
PID leftMotor(&left_wheel_measured_vel, &left_wheel_cmd, &left_wheel_cmd_vel, kp_l, ki_l, kd_l, DIRECT);


void setup() {
  // put your setup code here, to run once:

  pinMode(enableA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(enableB,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
 
  pinMode(Renco_Bc,INPUT);
  pinMode(Lenco_Bc,INPUT);

  attachInterrupt(digitalPinToInterrupt(Renco_Ac),right_encoder_callback,RISING);
  attachInterrupt(digitalPinToInterrupt(Lenco_Ac),left_encoder_callback,RISING);

  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  
  Serial.begin(115200);


}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available())
  {
    char chr = Serial.read();
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx=0;
      is_cmd_complete =false;

    }
    else if(chr =='l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx=0;

    }
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        digitalWrite(in1, HIGH - digitalRead(in1));
        digitalWrite(in2, HIGH - digitalRead(in2));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        digitalWrite(in3, HIGH - digitalRead(in3));
        digitalWrite(in4, HIGH - digitalRead(in4));
        is_left_wheel_forward = true;

      }

    }
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        digitalWrite(in1, HIGH - digitalRead(in1));
        digitalWrite(in2, HIGH - digitalRead(in2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        digitalWrite(in3, HIGH - digitalRead(in3));
        digitalWrite(in4, HIGH - digitalRead(in4));
        is_left_wheel_forward = false;

      }

    }
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete =true;

      }

      value_idx = 0;
      value[0]='0';
      value[1]='0';
      value[2]='.';
      value[3]='0';
      value[4]='0';
      value[5]='\0';
    }
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] =chr;
        value_idx++;

      }
    }


  }
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= intervel)
  {
    right_wheel_measured_vel = 10 * right_encoder_counter*(60.0/(90.0))*0.10472;
    left_wheel_measured_vel = 10 * left_encoder_counter*(60.0/(90.0))*0.10472;
    rightMotor.Compute();
    leftMotor.Compute();

    if(right_wheel_cmd_vel ==0.0)
    {
      right_wheel_cmd = 0;
    }
    if(left_wheel_cmd_vel ==0.0)
    {
      left_wheel_cmd = 0;
    }
    
    String encoder_reading = "r" + right_encoder_dir + String(right_wheel_measured_vel) + ",l" + left_encoder_dir + String(left_wheel_measured_vel) + ",";
    Serial.println(encoder_reading);
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(enableA, right_wheel_cmd);
    analogWrite(enableB, left_wheel_cmd);

  }


}


void right_encoder_callback(){
  right_encoder_counter++;
  if(digitalRead(Renco_Bc == HIGH)){
    right_encoder_dir = "p";
    }else{
    right_encoder_dir = "n";
    }
    
  }

  void left_encoder_callback(){
  left_encoder_counter++;
  if(digitalRead(Lenco_Bc == HIGH)){
    left_encoder_dir = "n";
    }else{
    left_encoder_dir = "p";
    }
    
  }

 















