#include "myRobot.h"

myRobot robot;
myRobot::Encoder encoder1(ENCODER1_A,ENCODER1_B);
myRobot::Encoder encoder2(ENCODER2_A,ENCODER2_B);

int pos = 0;
long prevT =0;
float eprev = 0;
float eintegral = 0;

void setup()
{
  Serial.begin(9600);
//   encoder_init();// initialise the encoder

}

void loop()
{ 

   int target = 1229;
  
   float kp = 1;
   float kd = 0;
   float ki = 0;

   long currT = micros();

   float deltaT = (float)(currT-prevT)/1.0e6;
   prevT = currT;

   int e = pos- target;

   float dedt = (e-eprev)/(deltaT);
   eintegral = eintegral + e*deltaT;

   float u = kp*e+ kd*dedt + ki*eintegral;

   float pwr = fabs(u);// absolute value 
   if (pwr>255)
   {
     pwr = 255;
   }

   bool dir = true;
   if (u<0){dir=false;}

   robot.moveM1(pwr,dir); // control 1 motor 

   eprev = e;

   Serial.print(target);
   Serial.print(" ");
   Serial.print(pos);
   Serial.println();
}

ISR(TIMER1_COMPA_vect)//timer1 interrupt 4Hz
{
  encoder1.show();
  encoder2.show(); 
}
