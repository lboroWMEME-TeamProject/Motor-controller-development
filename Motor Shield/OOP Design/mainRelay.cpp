#include "myRobot.h" // or #include "Relay.h"

Relay relay(RELAY1,RELAY2,RELAY3);// replace these with global definitions or any other pins

void setup()
{}

void loop()
{
  relay.On(); // turn ON ALL pins
  delay(100); // on for 100ms
  relay.Off();// turn OFF ALL pins
  delay(100);// off for 100ms
  
 // relay.setPin(1,"On"); // turn on pin # 1 = PIN10
 // relay.setPin(0,"off"); // turn off pin#0 = PIN7
}
