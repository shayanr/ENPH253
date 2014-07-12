#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>


void tape_follower (int Kp, int Kd, int thresh, int motorSpeed)
{
  //definig variables
  int left_s , right_s, P, D, G;
  int q,gain,count;
  int m=1;
  int recent_error=0;
  int error;
  int last_error=0;
  
  left_s = analogRead(0);                                  //Left QRD attached to analog 0
  right_s = analogRead(1);                                 //right QRD attached to analog 1

  if ((left_s>thresh)&&(right_s>thresh)) error = 0; 
  if ((left_s>thresh)&&(right_s<thresh)) error = -1; 
  if ((left_s<thresh)&&(right_s>thresh)) error = +1; 
  if ((left_s<thresh)&&(right_s<thresh)) 
  { 
    if (last_error>0) error = 3; 
    if (last_error<=0) error=-3 ; 
  }
  if (error != last_error)
  {
     recent_error=last_error;
     q = m;
     m = 1;
  }
  P=Kp*error; 
  D =(int)((float)Kd*(float)(error-recent_error)/(float)(q+m));
  gain=P+D;
  if (count=50)
  {
    LCD.clear();
    LCD.home();
    LCD.setCursor(0,0);
    LCD.print("l_s="); LCD.print(left_s);  LCD.print(","); LCD.print("r_s=");  LCD.print( right_s);// LCD.print(","); LCD.print("Kp="); LCD.print(Kp); LCD.print(","); LCD.print(Kd); LCD.print(",");LCD.print(P); LCD.print(D);
    delay(10);
    LCD.setCursor(0,1);
    //LCD.print("L_m="); LCD.print(-motorSpeed+gain); LCD.print(","); LCD.print("r_m="); LCD.print(motorSpeed+gain);
    count=0;
    delay(10);
  }
  count=count+1; 
  m=m+1;
  last_error=error;
  
  motor.speed(0, motorSpeed);    //right motor
  motor.speed(1, motorSpeed);     //left motor
  RCServo1.write (int(90+gain));    // turning the servo
  
}

