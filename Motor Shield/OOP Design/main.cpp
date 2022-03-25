#include "myRobot.h" // include the robot file 

myRobot robot; // initialise all pins
myRobot::Encoder encoder1(0); // insitialise the encoder1 pins
myRobot::Encoder encoder2(1); // insitialise the encoder2 pins

void setup()
{
  interrupt_init();// enable the encoder interrupts 
}

void loop()
{
 //  robot.move("forward",100);
}

ISR(TIMER1_COMPA_vect)//timer1 interrupt 4Hz
{
  encoder1.show(); // display RPM from both encoders 
  encoder2.show();
}
