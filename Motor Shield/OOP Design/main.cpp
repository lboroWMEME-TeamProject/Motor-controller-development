#include "myRobot.h"

myRobot robot;
myRobot::Encoder encoder1(ENCODER1_A,ENCODER1_B);
myRobot::Encoder encoder2(ENCODER2_A,ENCODER2_B);


void setup()
{
  encoder_init();// initialise the encoder 
}

void loop()
{ 
  robot.move("forward",100);
  delay(1000);
  robot.move("reverse",100);
  delay(3000);
  

}

ISR(TIMER1_COMPA_vect)//timer1 interrupt 4Hz
{
  encoder1.show();
  encoder2.show();
}
