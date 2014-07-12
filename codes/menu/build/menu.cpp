#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>

#include "WProgram.h"
#include <HardwareSerial.h>
void setup();
void loop();
int menu_next=0;
int menu_set=1;
int motorSpeed,Kp,Kd,thresh;
void setup() 
{
  portMode(0, INPUT);
 
}

void loop()
{
int count;
int error;
int last_error = 0;
int recent_error = 0;
int con;

  while(!startbutton())
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
}

