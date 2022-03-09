#include "myRobot.h"

Relay relay(7,10,14);// replace these with global definitions or any other pins

void setup()
{}

void loop()
{
  relay.On();
  delay(100);
  relay.Off();
  delay(100);
}
