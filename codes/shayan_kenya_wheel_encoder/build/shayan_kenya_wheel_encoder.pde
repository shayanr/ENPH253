 #include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

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
int target_velocityEEPROM=18;
int motorSpeedJumpEEPROM=20;
int IR_error_twoEEPROM=22;
int IR_differenceEEPROM=24;

// tape_follow() variables
int left_sensor=1  ;                                          //the analog input number of the left QRD sensor
int right_sensor=0;  


//the analog input number of the right QRD sensor
int rock_sensor=0;                                          //the digital input of the rock sensor
int left_s;
int right_s;
int q,gain,count=0;
int m=1;
int recent_error=0;
int error;
int last_error=0;
int servo_correction=90;                                //correction number get sent to the servo.
int max_turn=60;
int motor_gain = 0;
int left_motor=0;                                       //mototr pin number for left_motor  
int right_motor=1;                                      //motor pin number for right_motor  
int left_motor_offset=155;                              //left motor goes faster

boolean coming_down= false;

//arm variable
int artifact_number;  
int artifact_s=3;
int arm_motor=2;                                        //motor number for the arm
int arm_speed_up=900;
int arm_speed_down=300;
int arm_down=2;                                         //digital input of the arm_down push button

//IR variables
double I=0;
int rock_state=0;                                      //0= robot is not on the rock-pit    1= robot is on the rock-pit
int IR_Kp = getEepromValue(IR_KpEEPROM);
int IR_Kd = getEepromValue(IR_KdEEPROM);
int IR_motorSpeed = getEepromValue(IR_motorSpeedEEPROM);
int IR_thresh = getEepromValue(IR_threshEEPROM);          //Threshold to change the gains for the IR sensors
int IR_difference=getEepromValue(IR_differenceEEPROM);        //IR_differenceold for (left_IR - right_IR)
int left_IR;
int right_IR;
int left_IR1;                                                //Analog input for the left_IR1
int left_IR2;                                                //Analog input for the left_IR2
int right_IR1;                                               //Analog input for the right_IR1
int right_IR2;                                               //Analog input for the right_IR2
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


//sonar variables
int sonar_distance=0 ;
int sonar_state=0;
int sonar_height = 6;
int pulse_trig = 8;
int pulse_echo = 0;


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
int motorSpeedJump= getEepromValue(motorSpeedJumpEEPROM);
int target_velocity=  getEepromValue(target_velocityEEPROM);

//difining variables for IR_follower()


void setup()
{
 // portMode(1,OUTPUT);
  portMode(0,INPUT);
  pinMode(pulse_echo,INPUT);
  pinMode(pulse_trig,OUTPUT);
  pinMode(35,OUTPUT);                                  //servo
  attachInterrupt(encoder_pin, encoder, FALLING);                  //wheel encoder INT0
 
  RCServo0.attach(RCServo0Output);
  RCServo0.write(90);
  Serial.begin(9600);
  
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
      //  motor.speed(arm_motor,800);
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
    if(startbutton())
      return;
      
      while(digitalRead(menu_next)!= LOW && !startbutton())
    {
      displayValue("TargetSpeed =",target_velocity);
      if(digitalRead(menu_set) == LOW)
      {
         target_velocity = SetValue("Targetspeed =",1);
         Save(target_velocityEEPROM, target_velocity);
       }
    }
    delay(200);
    if(startbutton())
      return;
      
      
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("l_QRD_thresh=",left_QRD_thresh);
      if(digitalRead(menu_set) == LOW)
      {
        left_QRD_thresh = SetValue("l_QRD_thresh =",1);
        Save(left_QRD_threshEEPROM,left_QRD_thresh);      
      }
    }
   delay(200);
   
    if(startbutton())
      return;
    
        while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("R_QRD_thresh=",right_QRD_thresh);
      if(digitalRead(menu_set) == LOW)
      {
        right_QRD_thresh = SetValue("R_QRD_thresh=",1);
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
  if ((left_s>left_QRD_thresh)&&(right_s<right_QRD_thresh)) error = -error_one;     //Turn left
  if ((left_s<left_QRD_thresh)&&(right_s>right_QRD_thresh)) error = +error_one;     //Turn right
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
  
 ///////////////////Wheel encoder
  if(count==1)
  {
    count1=encoder_counter;
    t1 = millis();
  }
  
  if (count==2000)
  {
    count2=encoder_counter;
    t2=millis();
    velocity= ((count2 - count1)*(circumference)*1000.0)/( double((96.0*(t2-t1)))); 
    encoder_counter=0;
   if ( ((velocity - target_velocity) > 2) && (abs(gain)<(error_two *Kp) )) motorSpeed= motorSpeed - motorSpeedJump;
   else if ( ((target_velocity - velocity) > 2) && (abs(gain)<(error_two *Kp) )) motorSpeed= motorSpeed + motorSpeedJump;

    
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("l_s="); LCD.print(left_s);  LCD.print(","); LCD.print("r_s=");  LCD.print( right_s);
   // LCD.print("g=");LCD.print(gain);

    LCD.setCursor(0,1);
    LCD.print("velocity="); LCD.print(velocity);
  
    count=0;
    
   //Serial.print("t1-t2= "); Serial.print(t1-t2);Serial.print("\n");
   //Serial.print("count1-count2= "); Serial.print(count1-count2);Serial.print("\n");
   //Serial.print("speed= "); Serial.print(velocity);
   //Serial.print("\n--------------------------------------\n");
   //delay(1000);
  }
  
  count=count+1; 
  m=m+1;
  last_error=error;
  
 
    
  RCServo0.write (servo_correction);    // turning the servo
  motor.speed(left_motor, motorSpeed + motor_gain + left_motor_offset);    //left motor
  motor.speed(right_motor, motorSpeed - motor_gain);     //right motor
  
  motor_gain = 0;
  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IR_follow()
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IR_follow()
{
  left_IR = analogRead(left_IR1);                                  //Left IR 
  if (left_IR >= IR_thresh)
    left_IR= analogRead(left_IR2);
    
  right_IR = analogRead(right_IR1);                                //Right IR
  if (right_IR >= IR_thresh)
    right_IR= analogRead(right_IR2);
    
  if ( abs(left_IR - right_IR) < IR_difference ) IR_error=0;      //if 2 IR read almost the same value
  if ((left_IR - right_IR) > IR_difference) IR_error=-IR_error_one;          //if left reads more (Turn LEFT)
  if ((right_IR -left_IR )> IR_difference) IR_error=IR_error_one;            //if right reads more (Turn RIGHT)
  if ((left_IR < IR_difference) && (right_IR < IR_difference))    //if both are off 
  { 
    if (IR_last_error>0) IR_error = IR_error_two; 
    if (IR_last_error<=0) IR_error=- IR_error_two ; 
  }
  
  if (IR_error != IR_last_error)
  {
     IR_recent_error = IR_last_error;
     IR_q = IR_m;
     IR_m = 1;
  }
   IR_P= IR_Kp*IR_error; 
   IR_D= (int)((float)IR_Kd*(float)(IR_error - IR_recent_error)/(float)(IR_q + IR_m));
   IR_gain= (IR_P+IR_D)/(sqrt(left_IR + right_IR));
   
   IR_servo_correction= 90+IR_gain;
   IR_motor_gain = IR_gain;
   
   ///////////////////Wheel encoder
  if(IR_count==1)
  {
    count1=encoder_counter;
    t1 = millis();
  }
   if (IR_count=50)
  {
    count2=encoder_counter;
    t2=millis();
    velocity= ((count2 - count1)*(circumference)*1000.0)/( double((96.0*(t2-t1)))); 
    encoder_counter=0;
   if ( ((velocity - target_velocity) > 2) && (abs(IR_gain)<(IR_error_two *IR_Kp) )) IR_motorSpeed= IR_motorSpeed - motorSpeedJump;
   else if ( ((target_velocity - velocity) > 2) && (abs(IR_gain)<(IR_error_two *IR_Kp) )) IR_motorSpeed= IR_motorSpeed + motorSpeedJump;
   
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("l_s="); LCD.print(left_IR);  LCD.print(","); LCD.print("r_s=");  LCD.print( right_IR);
    delay(10);
    LCD.setCursor(0,1);
    //LCD.print("L_m="); LCD.print(-motorSpeed+gain); LCD.print(","); LCD.print("r_m="); LCD.print(motorSpeed+gain);
    IR_count=0;
  }
  
  IR_count = IR_count+1; 
  IR_m = IR_m + 1;
  IR_last_error = IR_error;
  
  
  
  RCServo0.write (IR_servo_correction);    // turning the servo
  motor.speed(left_motor, IR_motorSpeed + IR_motor_gain + left_motor_offset);    //left motor
  motor.speed(right_motor, IR_motorSpeed - IR_motor_gain);     //right motor
  
  motor_gain = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//turn()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turn()
{
  //turn full left back
  int turn_gain=90;
  int t_b=1000;                          //Time going back
  int t_f=1000;                          //Time going forward
  
  RCServo0.write (90 -max_turn);    // turning the servo
  motor.speed(right_motor, (-motorSpeed - turn_gain));    //right motor
  motor.speed(left_motor, (-motorSpeed + turn_gain));     //left motor
  delay(t_b);
  
  //turn full right forward
  RCServo0.write (90 + max_turn);    // turning the servo
  motor.speed(right_motor, (motorSpeed - turn_gain));     //right motor
  motor.speed(left_motor, (motorSpeed + turn_gain)  );    //left motor
  
  delay(t_f);
  
  last_error=0;
  error=0;
  
}
 
 
 /////////////////////////////////////////////////////////////////////////////////////////////
//encoder()
////////////////////////////////////////////////////////////////////////////////////////////
void encoder()
{
  encoder_counter++ ;
}
