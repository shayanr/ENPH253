#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>

// tape_follow() variables
int left_sensor=0;                                          //the analog input number of the left QRD sensor
int right_sensor=1;                                         //the analog input number of the right QRD sensor
int left_s;
int right_s;
int q,gain,count;
int m=1;
int recent_error=0;
int error;
int last_error=0;
int servo_correction=90;                                //correction number get sent to the servo.
int max_turn=80;

//Menu() variable
int menu_next=2;                                        //Digital input number for the menu_next button
int menu_set=3;                                         //Digital input number for the menu_set button 
int motorSpeed=280;
int Kp=10;
int Kd=0;
int Ki=10;
int QRD_thresh=250;
int P,D;

//difining variables for IR_follower()
float I;
int rock_state=0;                                      //0= robot is not on the rock-pit    1= robot is on the rock-pit
int IR_Kp, IR_Kd;
int left_IR;
int right_IR;
int IR_motorSpeed;
int IR_thresh;                                         //Threshold to change the gains for the IR sensors
int IR_difference;                                     //IR_differenceold for (left_IR - right_IR)

void setup()
{
  pinMode(0,INPUT);
  pinMode(35,OUTPUT);
  RCServo0.attach(RCServo0Output);
  RCServo0.write(90);
  
}
void loop() 
{
  while (!startbutton())
    Menu();
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 3");
  LCD.clear();
  LCD.setCursor(0,0);
  delay(1000);
  LCD.print("Starting in 2");
  delay(1000);
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 1");
  delay(1000);
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("GO !!!");
  LCD.clear();
  
  while (!stopbutton())
    tape_follow();
}


void Menu()
{
  delay(100);
    motor.speed(0,0);                             //motors are off 
    motor.speed(1,0);
    while(digitalRead(menu_next)!= LOW && !startbutton())
    {
      LCD.clear();
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("Speed =" ); LCD.print(motorSpeed); 
      LCD.setCursor(0,1);
      LCD.print("set=change val");
      delay(20);
      if(digitalRead(menu_set) == LOW)
      {
    
         while(digitalRead(menu_next) != LOW && !startbutton())
         {
           LCD.clear();
           LCD.setCursor(0,0);
           LCD.print("Speed ="); LCD.print(knob(6));
           motorSpeed = knob(6); 
           delay(20);
         }
       }
    }
    delay(300);
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      LCD.clear();
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("Kp =" ); LCD.print(Kp); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(10);
      if(digitalRead(menu_set) == LOW)
      {
        while(digitalRead(menu_next) != LOW && !startbutton())
        {
          LCD.clear();
          LCD.setCursor(0,0);
          LCD.print("Kp ="); LCD.print(knob(6));
          Kp = knob(6); 
          delay(10);
        }
      }
    }
    delay(300);
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      LCD.clear();
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("Kd = "); LCD.print(Kd); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(20);
      if(digitalRead(menu_set) == LOW)
      {
        while(digitalRead(menu_next)!= LOW && !startbutton())
        {
          LCD.clear();
          LCD.setCursor(0,0);
          LCD.print("Kd = "); LCD.print(knob(6));
          Kd = knob(6); 
          delay(20);
        }
      }
    }
    delay(300);
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      LCD.clear();
      LCD.home();
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
          LCD.print("Ki ="); LCD.print(knob(6));
          Ki = knob(6); 
          delay(10);
        }
      }
    }
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      LCD.clear();
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("QRD_thresh ="); LCD.print(QRD_thresh); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(20);
      if(digitalRead(menu_set) == LOW)
      {
        while(digitalRead(menu_next) != LOW && !startbutton())
        {
          LCD.clear();
          LCD.home();
          LCD.setCursor(0,0);
          LCD.print("QRD_thresh = "); LCD.print(knob(6));
          QRD_thresh = knob(6); 
          delay(20);
        }
      }
    }
  delay(300);
   while(digitalRead(menu_next) != LOW && !startbutton())
    {
      LCD.clear();
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("IR_Kp = "); LCD.print(IR_Kp); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(20);
      if(digitalRead(menu_set) == LOW)
      {
        while(digitalRead(menu_next)!= LOW && !startbutton())
        {
          LCD.clear();
          LCD.setCursor(0,0);
          LCD.print("IR_Kp = "); LCD.print(knob(6));
          IR_Kp = knob(6); 
          delay(20);
        }
      }
    }
    delay(300);
     while(digitalRead(menu_next) != LOW && !startbutton())
    {
      LCD.clear();
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("IR_Kd = "); LCD.print(IR_Kd); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(20);
      if(digitalRead(menu_set) == LOW)
      {
        while(digitalRead(menu_next)!= LOW && !startbutton())
        {
          LCD.clear();
          LCD.setCursor(0,0);
          LCD.print("IR_Kd = "); LCD.print(knob(6));
          IR_Kd = knob(6); 
          delay(20);
        }
      }
    }
    delay(300);
    delay(300);
    while(digitalRead(menu_next) != LOW && !startbutton())
    {
      LCD.clear();
      LCD.home();
      LCD.setCursor(0,0);
      LCD.print("IR_thresh ="); LCD.print(IR_thresh); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(20);
      if(digitalRead(menu_set) == LOW)
      {
        while(digitalRead(menu_next) != LOW && !startbutton())
        {
          LCD.clear();
          LCD.home();
          LCD.setCursor(0,0);
          LCD.print("IR_thresh = "); LCD.print(knob(6));
          IR_thresh = knob(6); 
          delay(20);
        }
      }
    }
    delay(200);
  }
  
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
//tape_Follow()
///////////////////////////////////////////////////////////////////////////////////////////////////////
void tape_follow()
{

  left_s = analogRead(left_sensor);                                  //Left QRD attached to analog 0
  right_s = analogRead(right_sensor);                                 //right QRD attached to analog 1

  if ((left_s>QRD_thresh)&&(right_s>QRD_thresh)) error = 0; 
  if ((left_s>QRD_thresh)&&(right_s<QRD_thresh)) error = -2; 
  if ((left_s<QRD_thresh)&&(right_s>QRD_thresh)) error = +2; 
  if ((left_s<QRD_thresh)&&(right_s<QRD_thresh)) 
  { 
    if (last_error>0) error = 4; 
    if (last_error<=0) error=-4; 
  }
  if (error != last_error)
  {
     recent_error=last_error;
     q = m;
     m = 1;
  }
  P=Kp*error; 

  D =(int)((float)Kd*(float)(error-recent_error)/(float)(q+m));
  if (abs(gain)<max_turn){
  I=(1/Ki)*(I+error);
  }
  gain=P+D+I;
  servo_correction= 90+gain;
  if (count=200)
  {
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("l_s="); LCD.print(left_s);  LCD.print(","); LCD.print("r_s=");  LCD.print( right_s);// LCD.print(","); LCD.print("Kp="); LCD.print(Kp); LCD.print(","); LCD.print(Kd); LCD.print(",");LCD.print(P); LCD.print(D);
    LCD.setCursor(0,1);
    LCD.print("C="); LCD.print(servo_correction);LCD.print("I="); LCD.print(I);// LCD.print(","); LCD.print("r_m="); LCD.print(motorSpeed+gain);
    delay(30);
    count=0;
  }
  count=count+1; 
  m=m+1;
  last_error=error;
  
  motor.speed(0, motorSpeed);    //right motor
  motor.speed(1, motorSpeed);     //left motor
  
  //servo_correction= 90+gain;
  if((servo_correction)>180) servo_correction=180;
  if (servo_correction<0) servo_correction=0; 
  RCServo0.write (servo_correction);    // turning the servo
  
}

 
