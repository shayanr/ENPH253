#include <phys253.h>
#include <Servo253.h>
#include <motor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

//variables to tune in menu
  int turn_gain=90;
  int t_b=1000;                          //Time going back
  int t_f=1000;                          //Time going forward
  int motorSpeed=375;
  
///motor variables  
  int left_motor=0;                                       //mototr pin number for left_motor  
  int right_motor=1;                                      //motor pin number for right_motor  
  int max_turn=60;
  
//Menu variable 
  int menu_next=6;                                        //Digital input number for the menu_next button
  int menu_set=7;                                          //Digital input number for the menu_set button 

//EEPROM
  int turn_gainEEPROM=22;
  int t_bEEPROM=24;
  int t_fEEPROM=26;

void setup()
{
 // portMode(1,OUTPUT);s
  portMode(0,INPUT);
  RCServo0.attach(RCServo0Output);
  RCServo0.write(90);
  Serial.begin(9600); 
}

void loop() 
{
  while (!startbutton())
    Menu();
  
  Countdown();
  
  while (!stopbutton())
  {
    turn();
    return;
  } 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Menu()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Menu()
{
  motor.stop_all();
  RCServo0.write(90);

 while(digitalRead(menu_next)!= LOW && !startbutton())
    {
      displayValue("turn_gain=",turn_gain);
      if(digitalRead(menu_set) == LOW)
      {
         turn_gain = SetValue("turn_gain=",2);
         Save(turn_gainEEPROM, turn_gain);
       }
    }
    delay(200);
    if(startbutton())
      return;
    
    while(digitalRead(menu_next) != LOW && !startbutton())
    { 
      displayValue("t_b=" ,t_b);
      if(digitalRead(menu_set) == LOW)
      {
        t_b= SetValue("t_b=", (0.5));
        Save(t_bEEPROM, t_b);
      }
    }
    delay(200);
    if(startbutton())
      return;


 while(digitalRead(menu_next) != LOW && !startbutton())
    {
      displayValue("t_f=", t_f);
      if(digitalRead(menu_set) == LOW)
      {
        t_f = SetValue("t_f =", (0.5));
        Save(t_fEEPROM,t_f);
      }
    }
    delay(200);
    if(startbutton())
      return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//turn()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turn()
{
  
  //turn full left back
  RCServo0.write (90 -max_turn);    // turning the servo
  motor.speed(right_motor, (-motorSpeed - turn_gain));    //right motor
  motor.speed(left_motor, (-motorSpeed + turn_gain));     //left motor
  delay(t_b);
  
  //turn full right forward
  RCServo0.write (90 + max_turn);    // turning the servo
  motor.speed(right_motor, (motorSpeed - turn_gain));     //right motor
  motor.speed(left_motor, (motorSpeed + turn_gain) );    //left motor
  
  delay(t_f);
  return;
  
  //last_error=0;
  //error=0;
  
}
 
 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//EEPROM stuff
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Save (int address, int value)
 {
   EEPROM.write(address, (char) (value & 0x00FF));
   EEPROM.write(address +1, static_cast<char>((value & 0xFF00) >> 8));
   return;
 }
 
 int getEepromValue(int address)
{
    int value = ((EEPROM.read(address)) | (EEPROM.read(address + 1) << 8));
    return value;
}
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//Countdown()
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void Countdown()
 {
  //3
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 3");
  delay(1000);
  
  //2
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 2");
  delay(1000);
  
  //1
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Starting in 1");
  delay(1000);
  
  //GO !!!
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("GO !!!");
  delay(500);
  LCD.clear();
  return;
 }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//displayValue(const char* display, int value)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  void displayValue(const char* display, int value)
  {
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print(display); LCD.print(value); 
      LCD.setCursor(0,1);
      LCD.print("set=change value");
      delay(20);
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SetValue(const char* display, int knobScale)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int SetValue(const char* display, double knobScale)
  {
    int val;
       while(digitalRead(menu_next) != LOW)
         {
           LCD.clear();
           LCD.setCursor(0,0);
           LCD.print(display); 
           LCD.setCursor(0,1);
           LCD.print(knob(6)/knobScale);
           val = knob(6)/knobScale; 
           delay(20);
         }
     return val;
  }
  
