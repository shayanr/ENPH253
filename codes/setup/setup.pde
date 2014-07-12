int menu_next=0;
int menu_set=1;

void setup() 
{
pinMode(24,OUTPUT);
pinMode(25,OUTPUT);
pinMode(menu_next, INPUT);          //
pinMode(menu_set, INPUT);
portMode(0, INPUT);
}


