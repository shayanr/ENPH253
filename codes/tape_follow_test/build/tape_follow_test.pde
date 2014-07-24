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
int motor_gain = 0;
int left_motor=0;                                       //mototr pin number for left_motor
int right_motor=1;                                      //motor pin number for right_motor

//sonar variables
int sonar_distance=0 ;
int sonar_state=0;
int sonar_height = 10;
int pulse_out = 8;
int pulse_in = 1;

//Menu() variable
int menu_next=2;                                        //Digital input number for the menu_next button
int menu_set=3;                                          //Digital input number for the menu_set button 
int motorSpeed=300;
int Kp=15;
int Kd=0;
float Ki=0.09;
int QRD_thresh=250;
int P,D;

//difining variables for IR_follower()
double I=0;
int rock_state=0;                                      //0= robot is not on the rock-pit    1= robot is on the rock-pit
int IR_Kp, IR_Kd;
int left_IR;
int right_IR;
int IR_motorSpeed;
int IR_thresh;                                         //Threshold to change the gains for the IR sensors
int IR_difference;                                     //IR_differenceold for (left_IR - right_IR)

void setup()
{
 // portMode(1,OUTPUT);
  //portMode(0,INPUT);
  pinMode(pulse_in,INPUT);
  pinMode(pulse_out,OUTPUT);
  
  pinMode(35,OUTPUT);                                  //servo
  analogWriteReset(pulse_out);
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
  delay(1000);
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 2");
  delay(1000);
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 1");
  delay(1000);
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("GO !!!");
  delay(500);
  LCD.clear();
  
  while (!stopbutton())
    tape_follow();
}


void Menu()
{
  delay(100);
    motor.speed(0,0);                             //motors are off 
    motor.speed(1,0);
    RCServo0.write(90);
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
          LCD.print("Kp ="); LCD.print(knob(6)/6);
          Kp = knob(6)/6; 
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
          LCD.print("Kd = "); LCD.print(knob(6)/6);
          Kd = knob(6)/6; 
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
          LCD.print("Ki ="); LCD.print((double)(0.1 + (knob(6)/1000.0)));
          Ki = (double) (0.1 + double(knob(6)/1000.0));
          delay(100);
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
  
  //if (abs(gain)<max_turn)
  //{
    I=(Ki)*(I+ (error*(q+m)));
  //}
  
  gain=P+D+I;
  if ((gain)>max_turn) gain = max_turn;
  if (gain < -max_turn) gain=-max_turn; 
  servo_correction= 90+gain;
    if (abs(error) == 4)
  {
    motor_gain = (0.5)*gain;
  }
  
  if (count=500)
  {
   
    //SONAR CODE
    
    //Makeing pulse

    digitalWrite(pulse_out,LOW);
    delayMicroseconds(2);
    digitalWrite(pulse_out, HIGH);
    delayMicroseconds(10);
    digitalWrite(pulse_out, LOW);
    //Reading Pulse
    sonar_distance = (pulseIn(pulse_in,HIGH))/58.2 ;
    
    //Check for impending clift
    if (sonar_distance > 2*sonar_height)
    {
       motor.speed(left_motor,-500);
       motor.speed(right_motor,-500);
       delay(200);
    }
   
   //Detect start of ramp   
   if (sonar_state == 0 && sonar_distance < sonar_height+1 )
   {
     motorSpeed = motorSpeed + 100;
     sonar_state = 1;
   }
   
   //Detect end of ramp
   if (sonar_state == 1 && sonar_distance > sonar_height+1 )
   {
     motorSpeed = motorSpeed - 100;
     sonar_state = 2;
   }
   
   //Detect Rocks
   if (sonar_state == 2 && sonar_distance < sonar_height+1)
   {
     rock_state = 1;
   }
   
  }
    
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    //LCD.print("l_s="); LCD.print(left_s);  LCD.print(","); LCD.print("r_s=");  LCD.print( right_s);// LCD.print(","); LCD.print("Kp="); LCD.print(Kp); LCD.print(","); LCD.print(Kd); LCD.print(",");LCD.print(P); LCD.print(D);
    //LCD.print("l_m="); LCD.print(motorSpeed + motor_gain);  LCD.print(","); LCD.print("r_m=");  LCD.print(  motorSpeed - motor_gain);// LCD.print(","); LCD.print("Kp="); LCD.print(Kp); LCD.print(","); LCD.print(Kd); LCD.print(",");LCD.print(P); LCD.print(D);
    //LCD.setCursor(0,1);
    //LCD.print("g="); LCD.print(gain);LCD.print("I="); LCD.print(I);// LCD.print(","); LCD.print("r_m="); LCD.print(motorSpeed+gain);
    LCD.print("dis="); LCD.print(sonar_distance);//LCD.print("I="); LCD.print(I);// LCD.print(","); LCD.print("r_m="); LCD.print(motorSpeed+gain);
    count=0;
  count=count+1; 
  m=m+1;
  last_error=error;
  

  motor.speed(left_motor, motorSpeed + motor_gain);    //left motor
  motor.speed(right_motor, motorSpeed - motor_gain);     //right motor
  
  RCServo0.write (servo_correction);    // turning the servo
  motor_gain = 0;
  
}

 
