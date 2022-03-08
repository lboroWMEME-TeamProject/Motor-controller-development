#include "myRobot.h"


 myRobot robot;
 myRobot::Encoder encoder(ENCODER1_A,ENCODER1_B);


void setup()
{
 Serial.begin(9600);
} 
void loop()
{
  robot.move("forward",100);
  encoder.show();
}
