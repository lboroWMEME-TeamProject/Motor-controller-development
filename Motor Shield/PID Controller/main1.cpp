#include "Control.h"

myRobot robot;
Controller pid1(0);
Controller pid2(1);

void setup()
{
  Serial.begin(9600);
  pid1.setCommand(true,1.75);  // set direction and distance in meters
  pid2.setCommand(true,1.75);
}

void loop()
{
  pid1.Compute();  
  pid2.Compute();
}
