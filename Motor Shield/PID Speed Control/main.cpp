#include "Control.hpp"

myRobot robot; // initialise all pins
Controller pid0(0);
Controller pid1(1);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
//   pid0.setAll(0.25,true,0.45,0.0675,0.60); 
//   pid1.setAll(0.25,true,1.30,0.095,0.65);
}
