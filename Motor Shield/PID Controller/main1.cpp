#include "myRobot.h"


myRobot robot;
Controller pid(0);

void setup()
{
  Serial.begin(9600);
  pid.setCommand(true,2);  // set direction and distance in meters
}

void loop()
{
  pid.Compute();  
  
}
