#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>

void IR_follower (int motorSpeed)
{
  //difining variables
  int Kp, Kd;
  int left_s , right_s, P, D, G;
  int q,gain,count;
  int m=1;
  int recent_error=0;
  int error;
  int last_error=0;
  int thresh;                                              //Threshold for (left_s - right_s)
  
  left_s = analogRead(0);                                  //Left IR attached to analog 0
  right_s = analogRead(1);                                 //right IR attached to analog 1
  
  if ( abs(left_s - right_s) < thresh ) error=0;
  if ((left_s - right_s) > thresh) error=-1;
  if ((right_s -left_s )> thresh) error=1;
  
  if (error != last_error)
  {
     recent_error=last_error;
     q = m;
     m = 1;
  }
   P=Kp*error; 
   D =(int)((float)Kd*(float)(error-recent_error)/(float)(q+m));
   gain= (P+D)/(sqrt(left_s + right_s));
   
   if (count=50)
  {
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("l_s="); LCD.print(left_s);  LCD.print(","); LCD.print("r_s=");  LCD.print( right_s);
    delay(10);
    LCD.setCursor(0,1);
    //LCD.print("L_m="); LCD.print(-motorSpeed+gain); LCD.print(","); LCD.print("r_m="); LCD.print(motorSpeed+gain);
    count=0;
  }
  
  count=count+1; 
  m=m+1;
  last_error=error;
  
  motor.speed(0, motorSpeed);    //right motor
  motor.speed(1, motorSpeed);     //left motor
  RCServo1.write (int(90+gain));    // turning the servo
  
}
   


