#include "Control.h"

myRobot robot; // initialise all pins
Controller pid1(0); // initialise PID for Motor1
Controller pid2(1);// initialise PID for Motor2

void setup()
{
  Serial.begin(9600);
  pid1.setCommand(true,1.5); // arg1 = direction, arg2= distance in meters
  pid1.setControl(5,0.5,0); // kp = 0.5, kd = 0.5, ki = 0

  pid2.setCommand(true,1.5); 
  pid2.setControl(5,0.5,0);  
}

void loop()
{
  pid1.Compute();
  pid2.Compute();
}
