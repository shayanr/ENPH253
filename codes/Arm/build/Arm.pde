#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
//Configuring the push botton inputs 
#define arm_sensor_down 2                //digital 2
#define arm_sensor 3                   //digital 3

volatile int arm_state=0;                //0=down, 1=up
int old_arm_state=0;


  
  
void setup()
{
   pinMode(0, INPUT);
  attachInterrupt(0, Arm_state, FALLING);
  Serial.begin(9600);
}

void loop()
{
  if(old_arm_state != arm_state)
  {
  Serial.print(arm_state);
  Arm(arm_state);
  old_arm_state= arm_state;
     arm_state=0;
  }
  Serial.print("Nothing has changed\n");
  Serial.print("----------------------------------------\n");
  delay(1000);
}


void Arm (int state)        //Moves the arm up and down
 {
  motor.stop(0);
  motor.stop(1);
  Serial.print("in ARM, state"); Serial.print(state);
  switch(state)
  {
    case 1:                                                  //state is up
     while( digitalRead(arm_sensor) != LOW )
    {
      Serial.print("Arm is coming up\n");
      delay(10);
    }
    break;
    case 0:                                                //state is low
     while(digitalRead(arm_sensor_down) != LOW)
    {
       Serial.print("arm is coming down\n");
       delay(10);
    }
     break;
  }
 }


void Arm_state()          //changes the arm_state - connected to interrupt0
 {
  arm_state= 1;
 }




