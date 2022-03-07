#include "myRobot.h"


 myRobot robot;
 myRobot::Encoder encoder;


void setup()
{
 Serial.begin(9600);
} 
void loop()
{
  //robot.move("forward",100);
  encoder.enableA();
}
