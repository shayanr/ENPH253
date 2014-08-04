#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
void encoder();
double encoder_counter = 0;
double count1,count2;
double t1,t2;
double velocity;
double circumference = 3.0;
int count=0;

void setup() 
{
  pinMode(0,INPUT);
  attachInterrupt(0, encoder, FALLING);
  Serial.begin(9600);

}

void loop() 
{
  while(!stopbutton())
  {
  motor.speed(0,400);
  motor.speed(1,400);
  if (count==1)
  {
  count1=encoder_counter;
  t1 = millis();
  }
  /*Serial.print("t1= ");Serial.print(t1);
  Serial.print("\n--------------------------------------\n");
  encoder_counter=0;
  */
  
  //delay(1000);
  if(count==30000)
  {
    count2=encoder_counter;
    t2=millis();
    velocity= ((count2 - count1)*(circumference)*1000.0)/( double((96.0*(t2-t1)))); 
    encoder_counter=0;
     // velocity= ((count2 - count1)*(circumference)*1000.0)/(24.0*(t2-t1)); 
   Serial.print("t1-t2= "); Serial.print(t1-t2);Serial.print("\n");
   Serial.print("count1-count2= "); Serial.print(count1-count2);Serial.print("\n");
   Serial.print("speed= "); Serial.print(velocity);
   Serial.print("\n--------------------------------------\n");
   delay(1000);
    //if ( (velocity - target_velocity) > (0.2* target_velocity) ) motorSpeed= motorSpeed - motorSpeedJump;
    //if ( (target_velocity - velocity) > (0.2* target_velocity) ) motorSpeed= motorSpeed + motorSpeedJump;

    count=0;
  }
  
  count++;
 /* Serial.print("t2= ");Serial.print(t2);
  Serial.print("\n--------------------------------------\n");
  */
  
  

}
}
/////////////////////////////////////////////////////////////////////////////////////////////
//encoder()
////////////////////////////////////////////////////////////////////////////////////////////
void encoder()
{
  encoder_counter++ ;
}

