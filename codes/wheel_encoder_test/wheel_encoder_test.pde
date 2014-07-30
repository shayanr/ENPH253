#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

int encoder_counter=0;
int count1,count2,t1,t2;
double velocity;
int circumference=3;

void setup() 
{
  pinMode(0,INPUT);
  attachInterrupt(0, encoder, FALLING);

}

void loop() 
{
  count1=encoder_counter;
  t1=millis();
  
  delay(1000);
  
  count2=encoder_counter;
  t2=millis();
  encoder_counter=0;
  
  velocity= ((count2 - count1)/24.0 )*(circumference)/(t1-t2);
  LCD.home();
  LCD.clear();
  LCD.print("speed="); LCD.print(velocity);

}

/////////////////////////////////////////////////////////////////////////////////////////////
//encoder()
////////////////////////////////////////////////////////////////////////////////////////////
void encoder()
{
  encoder_counter++ ;
}
