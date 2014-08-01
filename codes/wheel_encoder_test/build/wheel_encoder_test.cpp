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
long encoder_counter = 0;
long count1,count2;
long t1,t2;
double velocity;
double circumference = 3.0;

void setup() 
{
  pinMode(0,INPUT);
  attachInterrupt(0, encoder, FALLING);
  Serial.begin(9600);

}

void loop() 
{
  count1=encoder_counter;
  t1 = millis();
  /*Serial.print("t1= ");Serial.print(t1);
  Serial.print("\n--------------------------------------\n");
  encoder_counter=0;
  */
  
  //delay(1000);
  for (int i=0; i<400;i++)
  {
    LCD.setCursor(0,1);
    LCD.print(encoder_counter);
    delay(30);
  }
  
  count2=encoder_counter;
  t2=millis();
 /* Serial.print("t2= ");Serial.print(t2);
  Serial.print("\n--------------------------------------\n");
  */
  encoder_counter=0;
  
  velocity= ((count2 - count1)*(circumference)*1000.0)/( double((24.0*(t2-t1)))); 
 Serial.print("t1-t2= "); Serial.print(t1-t2);Serial.print("\n");
 Serial.print("count1-count2= "); Serial.print(count1-count2);Serial.print("\n");
 Serial.print("speed= "); Serial.print(velocity);
 Serial.print("\n--------------------------------------\n");
}

/////////////////////////////////////////////////////////////////////////////////////////////
//encoder()
////////////////////////////////////////////////////////////////////////////////////////////
void encoder()
{
  encoder_counter++ ;
}

