#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

//variables to tune in menu
  int left_motor_offset=155;                              //left motor goes faster
  
///motor variables  
  int left_motor=0;                                       //mototr pin number for left_motor  
  int right_motor=1;                                      //motor pin number for right_motor  
  int max_turn=60;
  int max_motorSpeed=500;
  
//Menu variable 
  int menu_next=6;                                        //Digital input number for the menu_next button
  int menu_set=7;                                          //Digital input number for the menu_set button 
  
  
//sonar variables
int sonar_distance=0 ;
int sonar_state=0;
int sonar_height = 3;
int pulse_trig = 8;
int pulse_echo = 1;
int sonar_counter=0;
int final_sonar_distance=0;
int sonar_counter2=0;

//EEPROM
int target_velocityEEPROM=18;
int motorSpeedJumpEEPROM=20;
int IR_error_twoEEPROM=22;
int IR_KpEEPROM=8;
int IR_KdEEPROM=10;
int IR_motorSpeedEEPROM=12;
int IR_threshEEPROM=14; 
int IR_differenceEEPROM=24;

//IR variables
double I=0;
int rock_state=0;                                              //0= robot is not on the rock-pit    1= robot is on the rock-pit
int IR_Kp = getEepromValue(IR_KpEEPROM);
int IR_Kd = getEepromValue(IR_KdEEPROM);
int IR_motorSpeed = getEepromValue(IR_motorSpeedEEPROM);
int IR_thresh = getEepromValue(IR_threshEEPROM);              //Threshold to change the gains for the IR sensors
int target_velocity=  getEepromValue(target_velocityEEPROM);
int motorSpeedJump= getEepromValue(motorSpeedJumpEEPROM);
int IR_difference=getEepromValue(IR_differenceEEPROM);        //IR_differenceold for (left_IR - right_IR)
int left_IR;
int right_IR;
int left_IR_low=3;                                                //Analog input for the left_IR_low
int left_IR_high=4;                                                //Analog input for the left_IR_high 
int right_IR_low=5;                                               //Analog input for the right_IR_low
int right_IR_high=6;                                               //Analog input for the right_IR_high
int IR_P, IR_D;
int IR_q,IR_gain,IR_count=0;
int IR_m=1;
int IR_recent_error=0;
int IR_error;
int IR_last_error=0;
int IR_error_one=1;
int IR_error_two=getEepromValue(IR_error_twoEEPROM);
int IR_servo_correction=0;
int IR_motor_gain;

//Wheel encoder
int encoder_pin=0;                                          //connected to INT0
long encoder_counter = 0.0;
long count1,count2;
long t1,t2;
double velocity=0.0;
double circumference = 10.0;

void setup()
{
 // portMode(1,OUTPUT);s
  portMode(0,INPUT);
  portMode(2,INPUT);
  RCServo0.attach(RCServo0Output);
  RCServo0.write(90);
  Serial.begin(9600); 
  attachInterrupt(encoder_pin, encoder, FALLING);                  //wheel encoder INT0
}

void loop() 
{
  while (!startbutton())
    Menu();
  
  Countdown();
  
  while (!stopbutton())
  {
    IR_follow();
  } 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Menu()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Menu()
{
  motor.stop_all();
  RCServo0.write(90);
  
  while(digitalRead(menu_next)!= LOW && !startbutton())
    {
      displayValue("IR_Speed=",IR_motorSpeed);
      if(digitalRead(menu_set) == LOW)
      {
         IR_motorSpeed = SetValue("Speed =",1);
         Save(IR_motorSpeedEEPROM, IR_motorSpeed);
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
        IR_Kp = SetValue("IR_Kp = ",1);
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
        IR_Kd = SetValue("IR_Kd = ",1);
        Save(IR_KdEEPROM,IR_Kd);
      }
    }
    delay(200);
    
     if(startbutton())
      return;
      
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("IR_thresh=",IR_thresh);
      if(digitalRead(menu_set) == LOW)
      {
        IR_thresh = SetValue("IR_thresh=",1);
        Save(IR_threshEEPROM,IR_thresh);
      }
    }
    
    delay(200);
    
     if(startbutton())
      return;
      
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("IR_diff=",IR_difference);
      if(digitalRead(menu_set) == LOW)
      {
        IR_difference = SetValue("IR_diff=",1);
        Save(IR_differenceEEPROM,IR_difference);
      }
    }
    
    delay(200);
    
     if(startbutton())
      return;
      
     while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("IR_error2=",IR_error_two);
      if(digitalRead(menu_set) == LOW)
      {
        IR_error_two = SetValue("IR_error 2=",120);
        Save(IR_error_twoEEPROM,IR_error_two);
      }
    }
    delay(200);
     delay(200);
    if(startbutton())
      return;
    
      while(digitalRead(menu_next)!= LOW && !startbutton())
    {
      displayValue("TargetSpeed=",target_velocity);
      if(digitalRead(menu_set) == LOW)
      {
         target_velocity = SetValue("Targetspeed=",20);
         Save(target_velocityEEPROM, target_velocity);
       }
    }
    delay(200);
        if(startbutton())
      return;
      
      
      while(digitalRead(menu_next)!= LOW && !startbutton())
    {
      displayValue("speedJump =",motorSpeedJump);
      if(digitalRead(menu_set) == LOW)
      {
         motorSpeedJump = SetValue("speedJump =",1);
         Save(motorSpeedJumpEEPROM, motorSpeedJump);
       }
    }
    delay(200);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IR_follow()
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IR_follow()
{
  left_IR = analogRead(left_IR_high);                                  //Left IR 
  if (left_IR >= IR_thresh)
    left_IR= analogRead(left_IR_low);
    
  right_IR = analogRead(right_IR_high);                                //Right IR
  if (right_IR >= IR_thresh)
    right_IR= analogRead(right_IR_low);
    
  if ( abs(left_IR - right_IR) < IR_difference ) IR_error=0;      //if 2 IR read almost the same value
  if ((left_IR - right_IR) > IR_difference) IR_error=-IR_error_one;          //if left reads more (Turn LEFT)
  if ((right_IR -left_IR )> IR_difference) IR_error=IR_error_one;            //if right reads more (Turn RIGHT)
  if ((left_IR < IR_difference) && (right_IR < IR_difference))    //if both are off 
  { 
    if (IR_last_error>0) IR_error = IR_error_two; 
    if (IR_last_error<=0) IR_error= -IR_error_two ; 
  }
  
  if (IR_error != IR_last_error)
  {
     IR_recent_error = IR_last_error;
     IR_q = IR_m;
     IR_m = 1;
  }
   IR_P= IR_Kp*IR_error; 
   IR_D= (int)((float)IR_Kd*(float)(IR_error - IR_recent_error)/(float)(IR_q + IR_m));
   IR_gain= (IR_P+IR_D)/sqrt(left_IR + right_IR);
   
   if ((IR_gain)>max_turn) IR_gain = max_turn;
   if (IR_gain < -max_turn) IR_gain= -max_turn;
   IR_servo_correction= 90+IR_gain;
   IR_motor_gain = IR_gain;
   
   ///////////////////Wheel encoder
  if(IR_count==1)
  {
    count1=encoder_counter;
    t1 = millis();
  }
  
  ///////sonar 
  if (sonar_counter==100)
  {
     digitalWrite(pulse_trig,LOW);
    delayMicroseconds(2);
    digitalWrite(pulse_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pulse_trig, LOW);
    
    //Reading Pulse
    sonar_distance = sonar_distance + (pulseIn(pulse_echo,HIGH))/58.2 ;
    sonar_counter=0;
    sonar_counter2 = sonar_counter2+1;
  }
  
  if (sonar_counter2==2)
  {
    final_sonar_distance = sonar_distance/3;
    sonar_distance==0;
    sonar_counter2=0;
    
    LCD.clear();
    LCD.home();
    LCD.print("dis=");LCD.print(final_sonar_distance);
    
  }
  
   if (IR_count=2000)
  {
    count2=encoder_counter;
    t2=millis();
    velocity= ((count2 - count1)*(circumference)*1000.0)/( double((96.0*(t2-t1)))); 
    encoder_counter=0;
   if ( ((velocity - target_velocity) > 2) && (abs(IR_gain)<(IR_error_two *IR_Kp))) IR_motorSpeed= IR_motorSpeed - motorSpeedJump;
   else if ( ((target_velocity - velocity) > 2) && (abs(IR_gain)<(IR_error_two *IR_Kp)) && (IR_motorSpeed < max_motorSpeed)) IR_motorSpeed= IR_motorSpeed + motorSpeedJump;
   
 //   LCD.clear();
  //  LCD.home();
   // LCD.setCursor(0,0);
  //  LCD.print("ls1="); LCD.print(analogRead(left_IR_low));  LCD.print(","); LCD.print("rs1=");  LCD.print(analogRead(right_IR_low));
   
    //LCD.print("dis=");LCD.print(sonar_distance);
    LCD.setCursor(0,1); 
 // LCD.print("Ls="); LCD.print(analogRead(left_IR_high)); LCD.print(","); LCD.print("rs="); LCD.print(analogRead(right_IR_high));
  LCD.print(left_IR); LCD.print(",");  LCD.print(right_IR);
    
  // Serial.print("ls1="); Serial.print(analogRead(left_IR_low));  Serial.print(","); Serial.print("rs1=");  Serial.print(analogRead(right_IR_low));
  // Serial.print("Ls2="); Serial.print(analogRead(left_IR_high)); Serial.print(","); Serial.print("rs2="); Serial.print(analogRead(right_IR_high));
  // Serial.print("\n------------------------------------------------------------------------");
    IR_count=0;
  }
  if (final_sonar_distance > 10*sonar_height)
    Turn();
  
  IR_count = IR_count+1; 
  sonar_counter =sonar_counter +1;
  IR_m = IR_m + 1;
  IR_last_error = IR_error;
  
  
  
  RCServo0.write (IR_servo_correction);    // turning the servo
  motor.speed(left_motor, IR_motorSpeed + IR_motor_gain + left_motor_offset);    //left motor
  motor.speed(right_motor, IR_motorSpeed - IR_motor_gain);     //right motor
  
  IR_motor_gain = 0;
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
  int SetValue(const char* display, double knobScale)
  {
    int val;
       while(digitalRead(menu_next) != LOW)
         {
           LCD.clear();
           LCD.setCursor(0,0);
           LCD.print(display); 
           LCD.setCursor(0,1);
           LCD.print(knob(7)/knobScale);
           val = knob(7)/knobScale; 
           delay(20);
         }
     return val;
  }
  
  
 /////////////////////////////////////////////////////////////////////////////////////////////
//encoder()
////////////////////////////////////////////////////////////////////////////////////////////
void encoder()
{
  encoder_counter++ ;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//turn()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Turn()
{
  //turn full left back
  int turn_gain=200;
  int t_b=2000;                          //Time going back
  int t_f=1500;                          //Time going forward
  int turn_motorSpeed=500;
  
  RCServo0.write (90 -max_turn);    // turning the servo
  motor.speed(right_motor, (-turn_motorSpeed - turn_gain));    //right motor
  motor.speed(left_motor, (-turn_motorSpeed + turn_gain));     //left motor
  delay(t_b);
  
  //turn full right forward
  RCServo0.write (90 + max_turn);    // turning the servo
  motor.speed(right_motor, (turn_motorSpeed - turn_gain));     //right motor
  motor.speed(left_motor, (turn_motorSpeed + turn_gain)  );    //left motor
  
  delay(t_f);
  
  IR_last_error=0;
  IR_error=0;
  
}
