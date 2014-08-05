#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

int left_motor=0;                                       //mototr pin number for left_motor  
int right_motor=1;                                      //motor pin number for right_motor 
int left_motor_offset=155;                              //left motor goes faster

void setup() 
{

}

void loop() 
{
  while (!startbutton())
  {
    LCD.home();
    LCD.print("press start");
    delay(30);
  }
  
   while (!stopbutton())
  {
   RCServo0.write (90);    // turning the servo
  motor.speed(left_motor, 400+ left_motor_offset);    //left motor
  motor.speed(right_motor, 400);     //right motor
  
  delay(2000);
  motor_stop();
  return;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////
//motor_stop()
//////////////////////////////////////////////////////////////////////////////////////////////
void motor_stop()
{
  motor.speed(left_motor,-500);
  motor.speed(right_motor,-500);
  delay(300);
  motor.stop(left_motor);
  motor.stop(right_motor);
}
