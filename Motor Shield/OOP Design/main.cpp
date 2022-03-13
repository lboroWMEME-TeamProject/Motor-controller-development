#include "myRobot.h"

//myRobot robot;
//myRobot::Encoder encoder(ENCODER1_A,ENCODER1_B);
//myRobot::Encoder encoder2(ENCODER2_A,ENCODER2_B);
//Relay relay(RELAY1,RELAY2,RELAY3);// replace these with global definitions or any other pins
EncoderCounter encoder;


void setup()
{
  Serial.begin (9600);  
}

void loop()
{
  encoder.showCount();
}
