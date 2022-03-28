#include "Control.h"

myRobot robot; // initialise all pins
Controller pid1(0);
Controller pid2(1);



void setup()
{
  Serial.begin(9600);
  pid1.setCommand(false,0.25); // arg1 = direction, arg2= speed in meteres/second 
  pid1.setControl(0.5,0,1.25);  // or try  pid1.setControl(0.25,0.025,1.00); 

  pid2.setCommand(false,0.25); 
  pid2.setControl(0.5,0,1.25);  
}

void loop()
{
//     pid1.Compute(); // start the PID controll MUST be placed inside  a while loop or fast interrupt
//     pid2.Compute();
}
