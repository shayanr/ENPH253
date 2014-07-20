#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>

//definig variables for tape follow
int left_s;                                          //the analog input number of the left QRD sensor
int right_s;                                         //the analog input number of the right QRD sensor
int P;
int D;
int G;
int q,gain,count;
int m=1;
int recent_error=0;
int error;
int last_error=0;

//menu variable
int menu_next;                                        //Digital input number for the menu_next button
int menu_set;                                         //Digital input number for the menu_set button 
int motorSpeed;
int Kp;
int Kd;
int thresh;

//Motor variables
int arm__state=0;                                      //The initial arm_state should be down
int arm_motor;                                         //The motor number that the arm motor is connected
int arm_speed;                                         //Speed pf the arm
int down_sensor;                                       //the analog input number of the down bumper switch
int up_sensor;                                         //The analog input number of the up bumper switch


//General variables
int artifact_number= 0;



void setup() 
{
  pinMode(0,INPUT);
  attachInterrupt(0, Arm_state, FALLING);
  
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
       if ( down_sensor != LOW)
         Arm(0);                                   //Brings the arm down
         
       if ( arm_state == 0)                       //No artifact is attached               
       {
         tape_follow();
       }
       else if ( up_sensor != LOW)                //checks if the arm is not up
         {
           Arm(1);                                //brings the arm down
           tape_follow();
         }
       else                                       //if the arm is up
       {
         artifact_number ++;                      //adds 1 to the number of artifacts
         if (down_sensor != LOW)                  //checks if the arm is not down 
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
      LCD.print("Thresh ="); LCD.print(thresh); 
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
          LCD.print("Thresh = "); LCD.print(knob(6));
          thresh = knob(6); 
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
  left_s = analogRead(left_s);                                  //Left QRD attached to analog 0
  right_s = analogRead(right_s);                                 //right QRD attached to analog 1

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
  }
  count=count+1; 
  m=m+1;
  last_error=error;
  
  motor.speed(0, motorSpeed);    //right motor
  motor.speed(1, motorSpeed);     //left motor
  RCServo1.write (int(90+gain));    // turning the servo
  
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
//IR_follower()
/////////////////////////////////////////////////////////////////////////////////////////////////////



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

