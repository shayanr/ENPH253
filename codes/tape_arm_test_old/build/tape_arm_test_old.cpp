#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

// tape_follow() variables
#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
void Save (int address, int value);
int getEepromValue(int address);
void Countdown();
void Menu();
void displayValue(const char* display, int value);
int SetValue(const char* display, int knobScale);
void tape_follow();
void turn();
int left_sensor=1;                                          //the analog input number of the left QRD sensor
int right_sensor=0;  

//the analog input number of the right QRD sensor
int rock_sensor=0;                                          //the digital input of the rock sensor
int left_s;
int right_s;
int q,gain,count;
int m=1;
int recent_error=0;
int error;
int last_error=0;
int servo_correction=90;                                //correction number get sent to the servo.
int max_turn=70;
int motor_gain = 0;
int left_motor=0;                                       //mototr pin number for left_motor  
int right_motor=1;                                      //motor pin number for right_motor  

boolean coming_down= false;

//arm variable
int artifact_number;
int artifact_s=3;
int arm_motor=2;                                        //motor number for the arm
int arm_speed_up=500;
int arm_speed_down=250;
int arm_down=2;                                         //digital input of the arm_down push button

//sonar variables
int sonar_distance=0 ;
int sonar_state=0;
int sonar_height = 6;
int pulse_trig = 8;
int pulse_echo = 0;


//EEPROM Adresses for Menu variables
int motorSpeedEEPROM=0;
int KpEEPROM=2;
int KdEEPROM=4;
int left_QRD_threshEEPROM=6;
int right_QRD_threshEEPROM=18;
int IR_KpEEPROM=8;
int IR_KdEEPROM=10;
int IR_motorSpeedEEPROM=12;
int IR_threshEEPROM=14; 
int error_twoEEPROM=16;

//Menu() variable
int menu_next=6;                                        //Digital input number for the menu_next button
int menu_set=7;                                          //Digital input number for the menu_set button 
int motorSpeed = getEepromValue(motorSpeedEEPROM);
int Kp = getEepromValue(KpEEPROM);
int Kd = getEepromValue(KdEEPROM);
float Ki=0.00;
int left_QRD_thresh=getEepromValue(left_QRD_threshEEPROM);
int right_QRD_thresh=getEepromValue(right_QRD_threshEEPROM);
int P,D;
int error_one = 1;
int error_two = getEepromValue(error_twoEEPROM);

//difining variables for IR_follower()
double I=0;
int rock_state=0;                                      //0= robot is not on the rock-pit    1= robot is on the rock-pit
int IR_Kp = getEepromValue(IR_KpEEPROM);
int IR_Kd = getEepromValue(IR_KdEEPROM);
int left_IR;
int right_IR;
int IR_motorSpeed = getEepromValue(IR_motorSpeedEEPROM);
int IR_thresh = getEepromValue(IR_threshEEPROM);          //Threshold to change the gains for the IR sensors
int IR_difference;                                     //IR_differenceold for (left_IR - right_IR)

void setup()
{
 // portMode(1,OUTPUT);
  portMode(0,INPUT);
  pinMode(pulse_echo,INPUT);
  pinMode(pulse_trig,OUTPUT);
  pinMode(35,OUTPUT);                                  //servo
 
  RCServo0.attach(RCServo0Output);
  RCServo0.write(90);
  
}
void loop() 
{
  
  while (!startbutton())
    Menu();
    
  Countdown();
  
  while (!stopbutton())
  {
    if (digitalRead(artifact_s) != LOW)                      //No artifact is attached
    {
      tape_follow();
      if(digitalRead(arm_down)!= LOW)                        //arm is not down
      {
        motor.speed(arm_motor,1000);
        motor.speed(arm_motor,arm_speed_down);                    //brings the arm down
        tape_follow();
      }
      else                                                   //arm is down and there is no artifact attached 
      {
        tape_follow();
        motor.stop(arm_motor);
      }
    }
    
    else                                                    //artifact is attached
    {
      artifact_number++;
       motor.speed(arm_motor,-1000); 
      motor.speed(arm_motor,-arm_speed_up);                        //brings the arm up
      tape_follow();
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//EEPROM stuff
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Save (int address, int value)
 {
   EEPROM.write(address, (char) (value & 0x00FF));
   EEPROM.write(address +1, static_cast<char>((value & 0xFF00) >> 8));
   return;
 }
 
 int getEepromValue(int address)
{
    int value = ((EEPROM.read(address)) | (EEPROM.read(address + 1) << 8));
    return value;
}
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//Countdown()
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void Countdown()
 {
  //3
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 3");
  delay(1000);
  
  //2
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 2");
  delay(1000);
  
  //1
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 1");
  delay(1000);
  
  //GO !!!
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("GO !!!");
  delay(500);
  LCD.clear();
  return;
 }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Menu()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Menu()
{
    delay(100);
    motor.speed(0,0);                             //motors are off 
    motor.speed(1,0);
    motor.speed(2,0);
    RCServo0.write(90);
        
    while(digitalRead(menu_next)!= LOW && !startbutton())
    {
      displayValue("Speed =",motorSpeed);
      if(digitalRead(menu_set) == LOW)
      {
         motorSpeed = SetValue("Speed =",1);
         Save(motorSpeedEEPROM, motorSpeed);
       }
    }
    delay(200);
    if(startbutton())
      return;
    
    while(digitalRead(menu_next) != LOW && !startbutton())
    { 
      displayValue("Kp = " ,Kp);
      if(digitalRead(menu_set) == LOW)
      {
        Kp = SetValue("Kp =",6);
        Save(KpEEPROM, Kp);
      }
    }
    delay(200);
    if(startbutton())
      return;
    
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("Kd = ",Kd);
      if(digitalRead(menu_set) == LOW)
      {
        Kd = SetValue("Kd = ",6);
        Save(KdEEPROM,Kd);
      }
    }
    delay(200);
    if(startbutton())
      return;
    
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print("Ki =" ); LCD.print(Ki); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(10);
      if(digitalRead(menu_set) == LOW)
      {
        while(digitalRead(menu_next) != LOW && !startbutton())
        {
          LCD.clear();
          LCD.setCursor(0,0);
          LCD.print("Ki ="); LCD.print((double)((knob(6)/500.0)));
          Ki = (double) (double(knob(6)/500.0));
          delay(10);
        }
      }
    }
    delay(200);
    
    if(startbutton())
      return;
      
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("l_QRD_thresh =",left_QRD_thresh);
      if(digitalRead(menu_set) == LOW)
      {
        left_QRD_thresh = SetValue("l_QRD_thresh = ",1);
        Save(left_QRD_threshEEPROM,left_QRD_thresh);      
      }
    }
   delay(200);
   
    if(startbutton())
      return;
    
        while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("R_QRD_thresh =",right_QRD_thresh);
      if(digitalRead(menu_set) == LOW)
      {
        right_QRD_thresh = SetValue("R_QRD_thresh = ",1);
        Save(right_QRD_threshEEPROM,right_QRD_thresh);      
      }
    }
   delay(200);
   
    if(startbutton())
      return;
      
   while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("IR_Kp = ",IR_Kp);
      if(digitalRead(menu_set) == LOW)
      {
        IR_Kp = SetValue("IR_Kp = ",6);
        Save(IR_KpEEPROM,IR_Kp);
      }
    }
    delay(200);
    
     if(startbutton())
      return;
      
     while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("IR_Kd = ",IR_Kd);
      
      if(digitalRead(menu_set) == LOW)
      {
        IR_Kd = SetValue("IR_Kd = ",6);
        Save(IR_KdEEPROM,IR_Kd);
      }
    }
    delay(200);
    
     if(startbutton())
      return;
      
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("IR_thresh =",IR_thresh);
      if(digitalRead(menu_set) == LOW)
      {
        IR_thresh = SetValue("IR_thresh = ",1);
        Save(IR_threshEEPROM,IR_thresh);
      }
    }
    
    delay(200);
    
     if(startbutton())
      return;
      
     while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("Error 2=",error_two);
      if(digitalRead(menu_set) == LOW)
      {
        error_two = SetValue("Error 2=",120);
        Save(error_twoEEPROM,error_two);
      }
    }
    delay(200);
  }
  
  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//displayValue(const char* display, int value)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  void displayValue(const char* display, int value)
  {
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(display); LCD.print(value); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(20);
  }

  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SetValue(const char* display, int knobScale)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int SetValue(const char* display, int knobScale)
  {
    int val;
       while(digitalRead(menu_next) != LOW)
         {
           LCD.clear();
           LCD.setCursor(0,0);
           LCD.print(display); 
           LCD.setCursor(0,1);
           LCD.print(knob(6)/knobScale);
           val = knob(6)/knobScale; 
           delay(20);
         }
     return val;
  }
  
  
///////////////////////////////////////////////////////////////////////////////////////////////////////
//tape_Follow()
///////////////////////////////////////////////////////////////////////////////////////////////////////
void tape_follow()
{

  left_s = analogRead(left_sensor);                                  //Left QRD attached to analog 0
  right_s = analogRead(right_sensor);                                 //right QRD attached to analog 1

  if ((left_s>left_QRD_thresh)&&(right_s>right_QRD_thresh)) error = 0; 
  if ((left_s>left_QRD_thresh)&&(right_s<right_QRD_thresh)) error = -error_one; 
  if ((left_s<left_QRD_thresh)&&(right_s>right_QRD_thresh)) error = +error_one; 
  if ((left_s<left_QRD_thresh)&&(right_s<right_QRD_thresh)) 
  { 
    if (last_error>0) error = error_two; 
    if (last_error<=0) error=-error_two; 
  }
  
  if (error != last_error)
  {
     recent_error=last_error;
     q = m;
     m = 1;
  }
  
  P=Kp*error; 

  D =(int)((float)Kd*(float)(error-recent_error)/(float)(q+m));
  
  if (abs(gain)<max_turn)
  {
    I=(Ki)*(I+ (error*(q+m)));
  }
  
  gain=P+D+I;
  if ((gain)>max_turn) gain = max_turn;
  if (gain < -max_turn) gain=-max_turn; 
  servo_correction= 90+gain;

  motor_gain = gain;
  
  if (count=500)
  {
   
 /*   //SONAR CODE
    //Makeing pulse

    digitalWrite(pulse_trig,LOW);
    delayMicroseconds(2);
    digitalWrite(pulse_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pulse_trig, LOW);
    
    //Reading Pulse
    sonar_distance = (pulseIn(pulse_echo,HIGH))/58.2 ;
    
    if (sonar_distance > 10*sonar_height)
      {
         motor.speed(left_motor,-500);
         motor.speed(right_motor,-500);
         delay(1000);
      }
      
     if (digitalRead(rock_sensor)== LOW)                                                      //if the robot has hit the rocks
     {
       coming_down= true;
       turn();
     }      
     
    //Check for impending clift
    if (coming_down == false)
   {  
     //Detect start of ramp   
     if (sonar_state == 0 && sonar_distance < sonar_height-5 )
     {
       motorSpeed = motorSpeed + 100;
       sonar_state = 1;
     }
   
     //Detect end of ramp
     if (sonar_state == 1 && sonar_distance > sonar_height+5 )
     {
       motorSpeed = motorSpeed - 100;
       sonar_state = 2;
     }
   
    //Detect Rocks
    // if (sonar_state == 2 && sonar_distance < sonar_height+1)
    //{
    //  rock_state = 1;
    //}
   }
   else                          //detects when we are coming down 
   {
     if (sonar_state == 2 && sonar_distance > sonar_height+50 )
     {
       motorSpeed = motorSpeed - 100;
       sonar_state = 1;
     }
   
     //Detect end of ramp
     if (sonar_state == 1 && sonar_distance > sonar_height+50 )
     {
       motorSpeed = motorSpeed + 100;
       sonar_state = 2;
     }
   
     //Detect Rocks
    // if (sonar_state == 2 && sonar_distance < sonar_height+1)
     //{
      // rock_state = 1;
     //}
     
   }
   
   
   */
    
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("l_s="); LCD.print(left_s);  LCD.print(","); LCD.print("r_s=");  LCD.print( right_s);// LCD.print(","); LCD.print("Kp="); LCD.print(Kp); LCD.print(","); LCD.print(Kd); LCD.print(",");LCD.print(P); LCD.print(D);
    //LCD.print("l_m="); LCD.print(motorSpeed + motor_gain);  LCD.print(","); LCD.print("r_m=");  LCD.print(  motorSpeed - motor_gain);// LCD.print(","); LCD.print("Kp="); LCD.print(Kp); LCD.print(","); LCD.print(Kd); LCD.print(",");LCD.print(P); LCD.print(D);
    LCD.setCursor(0,1);
    //LCD.print("g="); LCD.print(gain);LCD.print("I="); LCD.print(I);// LCD.print(","); LCD.print("r_m="); LCD.print(motorSpeed+gain);
    LCD.print("dis="); LCD.print(sonar_distance);LCD.print("g="); LCD.print(gain);// LCD.print(","); LCD.print("r_m="); LCD.print(motorSpeed+gain);
    count=0;
  }
  count=count+1; 
  m=m+1;
  last_error=error;
  

  RCServo0.write (servo_correction);    // turning the servo
  motor.speed(left_motor, motorSpeed + motor_gain);    //left motor
  motor.speed(right_motor, motorSpeed - motor_gain);     //right motor
  
  motor_gain = 0;
  
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//turn()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turn()
{
  int turn_gain=90;
  RCServo0.write (90 + max_turn);    // turning the servo
  motor.speed(left_motor, (-motorSpeed - turn_gain));    //left motor
  motor.speed(right_motor, (-motorSpeed + turn_gain));     //right motor
  delay(1000);
  
  RCServo0.write (90 - max_turn);    // turning the servo
  motor.speed(left_motor, (motorSpeed-turn_gain) );    //left motor
  motor.speed(right_motor, (motorSpeed+turn_gain));     //right motor
  delay(1000);
  
  last_error=0;
  error=0;
  
}

