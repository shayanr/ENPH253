#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
  int arm_down=2;  
void setup() 
{

  portMode(0,INPUT);
}

void loop() 
{
  LCD.clear();
  LCD.home();
  LCD.print(digitalRead(arm_down));
  delay(50);
}

