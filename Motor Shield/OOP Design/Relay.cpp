#include "Arduino.h"
#include "Relay.h"


Relay::Relay(int a,int b,int c)
{
  pins[0]=a;
  pins[1]=b;
  pins[2]=c;


  pinMode(pins[0], OUTPUT); 
  pinMode(pins[1], OUTPUT); 
  pinMode(pins[2], OUTPUT); 
}

void Relay::On() // LOW= HIGH due to incorrect wiring 
{
  digitalWrite(pins[0], LOW); 
  digitalWrite(pins[1], LOW);  
  digitalWrite(pins[2], LOW); 

}

void Relay::Off()
{
  digitalWrite(pins[0], HIGH); 
  digitalWrite(pins[1], HIGH);  
  digitalWrite(pins[2], HIGH); 
}
