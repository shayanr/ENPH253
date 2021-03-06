///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Phys 253 robotic lab
// summer 2014
// Author: Shayan Rezaiezadeh
//Email: shayan@alumni.ubc.ca
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>

// tape_follow() variables
int left_sensor;                                          //the analog input number of the left QRD sensor
int right_sensor;                                         //the analog input number of the right QRD sensor
int left_s;
int right_s;
int q,gain,count;
int m=1;
int recent_error=0;
int error;
int last_error=0;
int servo_correction;                                //correction number get sent to the servo.

//Menu() variable
int menu_next;                                        //Digital input number for the menu_next button
int menu_set;                                         //Digital input number for the menu_set button 
int motorSpeed;
int Kp;
int Kd;
int QRD_thresh;
int P,D;

//Arm() variables
int arm_state=0;                                       //The initial arm_state should be down
int arm_motor;                                         //The motor number that the arm motor is connected
int arm_speed;                                         //Speed pf the arm
int down_sensor;                                       //the analog input number of the down bumper switch
int up_sensor;                                         //The analog input number of the up bumper switch

 //difining variables for IR_follower()
int rock_state=0;                                      //0= robot is not on the rock-pit    1= robot is on the rock-pit
int IR_Kp, IR_Kd;
int left_IR;
int right_IR;
int IR_motorSpeed;
int IR_thresh;                                         //Threshold to change the gains for the IR sensors
int IR_difference;                                     //IR_differenceold for (left_IR - right_IR)
int left_IR1;                                          //Analog input for the left_IR1
int left_IR2;                                          //Analog input for the left_IR2
int right_IR1;                                         //Analog input for the right_IR1
int right_IR2;                                         //Analog input for the right_IR2
int IR_P, IR_D;
int IR_q,IR_gain,IR_count;
int IR_m=1;
int IR_recent_error=0;
int IR_error;
int IR_last_error=0;
int IR_servo_correction;


//General variables
int artifact_number= 0;



void setup() 
{
  pinMode(0,INPUT);
  pinMode(35,OUTPUT);                          //set servo_0 to output
  attachInterrupt(0, Arm_state, FALLING);
  attachInterrupt(1, Rock_state, FALLING);
  
  /////////////////////////////////////////////set up motors left_motor,right_motor,arm_motor
  
}

void loop() 
{
  while (!startbutton())
  Menu();
  
  while (!stopbutton())
  {
     while(artifact_number <= 3)
     {
       if (rock_state==1)
         IR_follow();
       else
       {
         if ( digitalRead(down_sensor) != LOW)
           Arm(0);                                   //Brings the arm down
         
         if ( arm_state == 0)                       //No artifact is attached               
         {
           tape_follow();
         }
         else if ( digitalRead(up_sensor) != LOW)                //checks if the arm is not up
           {
             Arm(1);                                //brings the arm down
             tape_follow();
           }
         else                                       //if the arm is up
         {
           artifact_number ++;                      //adds 1 to the number of artifacts
           if (digitalRead(down_sensor) != LOW)                  //checks if the arm is not down 
           {
             Arm(0);                                //brings the arm down
             tape_follow();  
           }
           else                                      //arm is down
           {
             arm_state=0;                            //changes the arm_state to down
             motor.stop(arm_motor);                  //stops the arm motor
           }
         }
        }
       }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Menu()
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void Menu()
{
  delay(100);
    motor.speed(0,0);                             //motors are off 
    motor.speed(1,0);
    while(digitalRead(menu_next)!= LOW)
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
    
         while(digitalRead(menu_next) != LOW)
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
    while(digitalRead(menu_next) != LOW)
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
        while(digitalRead(menu_next) != LOW)
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
    while(digitalRead(menu_next) != LOW)
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
        while(digitalRead(menu_next)!= LOW)
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
    while(digitalRead(menu_next) != LOW)
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
        while(digitalRead(menu_next) != LOW)
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
   while(digitalRead(menu_next) != LOW)
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
        while(digitalRead(menu_next)!= LOW)
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
     while(digitalRead(menu_next) != LOW)
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
        while(digitalRead(menu_next)!= LOW)
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
    while(digitalRead(menu_next) != LOW)
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
        while(digitalRead(menu_next) != LOW)
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



/////////////////////////////////////////////////////////////////////////////////////////////////////
//Arm(int state) 
// state=0 -> brings the arm down
// state=1 -> brings the arm up
/////////////////////////////////////////////////////////////////////////////////////////////////////
void Arm(int state)
{
  switch(state)
  {
    case 0:
    motor.speed(arm_motor,- arm_speed);          //brings the arm down
    break;
    case 1:
    motor.speed(arm_motor,arm_speed);
    break;
  }
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//tape_Follow()
///////////////////////////////////////////////////////////////////////////////////////////////////////
void tape_follow()
{

  left_s = analogRead(left_sensor);                                  //Left QRD attached to analog 0
  right_s = analogRead(right_sensor);                                 //right QRD attached to analog 1

  if ((left_s>QRD_thresh)&&(right_s>QRD_thresh)) error = 0; 
  if ((left_s>QRD_thresh)&&(right_s<QRD_thresh)) error = -1; 
  if ((left_s<QRD_thresh)&&(right_s>QRD_thresh)) error = +1; 
  if ((left_s<QRD_thresh)&&(right_s<QRD_thresh)) 
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
  }
  count=count+1; 
  m=m+1;
  last_error=error;
  
  motor.speed(0, motorSpeed);    //right motor
  motor.speed(1, motorSpeed);     //left motor
  
  servo_correction= 90+gain;
  if((servo_correction)>180) servo_correction=180;
  if (servo_correction<0) servo_correction=0; 
  RCServo0.write (servo_correction);    // turning the servo
  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//Arm_state() 
//changes the arm_state - connected to interrupt0
//////////////////////////////////////////////////////////////////////////////////////////////////////
void Arm_state()          
 {
  arm_state= 1;
 }
 

//////////////////////////////////////////////////////////////////////////////////////////////////////
//Rock_state()
//detects the rock-pit - connected to interrupt1
/////////////////////////////////////////////////////////////////////////////////////////////////////
 void Rock_state()
 {
   if (artifact_number==3)
   rock_state=1;
 }
 
//////////////////////////////////////////////////////////////////////////////////////////////////////
//IR_follower()
/////////////////////////////////////////////////////////////////////////////////////////////////////
void IR_follow ()
{ 
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
  
  motor.speed(0, IR_motorSpeed);    //right motor
  motor.speed(1, IR_motorSpeed);     //left motor
  
  IR_servo_correction= 90+gain;
  if((IR_servo_correction)>180) IR_servo_correction=180;
  if (IR_servo_correction<0) IR_servo_correction=0; 
  RCServo0.write (int(IR_servo_correction));              // turning the servo  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//stop_motor()                  //can be implemented if needed 
///////////////////////////////////////////////////////////////////////////////////////////////////////

/*void stop_motor(arrifact_number)
{
  switch(artifact_nember)
  {
    case 0:
    break;
    
    case 1:
    break;
    
    case 2:
    break;
    
    case 3:
    break;
    
    case 4:
    break;
  }
}


*/

