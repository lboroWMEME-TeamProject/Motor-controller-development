#include "Relay.h" 
Relay relay(RELAY1,RELAY2,RELAY3);// replace these with global definitions or any other pins

void setup()
{}

void loop()
{
//  relay.On(); // turn ON ALL pins
//  delay(100); // on for 100ms
//  relay.Off();// turn OFF ALL pins
// delay(100);// off for 100ms
  
  relay.setPin(1,"off"); // turn off pin # 1 = PIN10
//relay.setPin(0,"ON"); // turn on pin#0 = PIN7
//  relay.setPin(2,"off"); // turn off pin#2 = PIN14
}
