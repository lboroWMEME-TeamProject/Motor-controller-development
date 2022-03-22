#include "myRobot.h" // or #include "Relay.h"

Relay relay(RELAY1,RELAY2,RELAY3);// replace these with global definitions or any other pins

void setup()
{}

void loop()
{
  relay.On();
  delay(100);
  relay.Off();
  delay(100);
}
