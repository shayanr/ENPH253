#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>

void IR_follower (int motorSpeed)
{
 //difining variables for IR_follower()
 int IR_Kp, IR_Kd;
 int left_IR;
 int right_IR;
 int IR_thresh;                                               //Threshold to change the gains for the IR sensors
 int IR_difference;                                           //IR_differenceold for (left_IR - right_IR)
 int left_IR1;                                                //Analog input for the left_IR1
 int left_IR2;                                                //Analog input for the left_IR2
 int right_IR1;                                               //Analog input for the right_IR1
 int right_IR2;                                               //Analog input for the right_IR2
 int IR_P, IR_D, G;
 int IR_q,IR_gain,IR_count;
 int IR_m=1;
 int IR_recent_error=0;
 int IR_error;
 int IR_last_error=0;
 
  
  left_IR = analogRead(left_IR1);                                  //Left IR1 
  if (left_IR >= IR_thresh)
    left_IR= analogRead(left_IR2);
    
  right_IR = analogRead(right_IR1);                                //Right IR2
  if (right_IR >= IR_thresh)
    right_IR= analogRead(right_IR2);
    
  if ( abs(left_IR - right_IR) < IR_difference ) IR_error=0;
  if ((left_IR - right_IR) > IR_difference) IR_error=-1;
  if ((right_IR -left_IR )> IR_difference) IR_error=1;
  if ((left_IR < IR_difference) && (right_IR < IR_difference))
  { 
    if (IR_last_error>0) IR_error = 3; 
    if (IR_last_error<=0) IR_error=-3 ; 
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
   
   if (IR_count=50)
  {
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
  
  motor.speed(0, motorSpeed);    //right motor
  motor.speed(1, motorSpeed);     //left motor
  RCServo1.write (int(90 + IR_gain));    // turning the servo  
}
   


