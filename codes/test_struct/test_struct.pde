extern"C"
{
typedef struct global
{
 int x=2;
 int y=3; 
}

struct global variable;
}

void setup()
{
  Serial.begin(9600);
}
void loop() 
{
delay(1000);
function(variable);
Serial.print(variable.x);
Serial.print'\n-------------------');
}

int function(global variable)
{
  return variable.x+2;
}
