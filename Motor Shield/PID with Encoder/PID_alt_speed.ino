#include "Control.hpp"

myRobot robot; // initialise all pins
Controller pid0(0); // create PID object for Motor 0
Controller pid1(1);// create PID object for Motor 1

//myRobot::Encoder encoder1(0); // insitialise the encoder1 pins, used for printing encoder speed
//myRobot::Encoder encoder2(1); // insitialise the encoder2 pins


void setup()
{
  Serial.begin(115200);
  pid0.setCommand(false,0.50); // set direction and speed for PID0
  pid0.setControl(0.275,0.0850,0.55);// set Kp,Kd and Ki
 
  
  pid1.setCommand(false,0.25); // set direction and speed for PID1
  pid1.setControl(0.30,0.0750,0.75);// set Kp,Kd and Ki

//  encoder_init();  function used to initialise the timer based interrupt 
}

void loop()
{
//   pid0.Compute();// start computing and driving PID signal 
//   pid1.Compute();


// alternate function that does not require setCommand() and setControl member functions 
 //  pid0.setAll(0.75,false,0.275,0.0850,0.55); // arguments: target in m/s,direction,kp,kd,ki
 //  pid1.setAll(0.75,false,0.30,0.0750,0.75); 

}


/*
ISR(TIMER1_COMPA_vect)//timer1 interrupt 4Hz, interrupt hander function 
{
//  encoder1.show();// print the real-time encoder speed
//  encoder2.show();
}
*/
