#include "Arduino.h"
#include <util/atomic.h>
#include "Control.hpp"

myRobot::myRobot()
{
  //Setup Motor1
  pinMode(dir1, OUTPUT); //Initiates Motor Channel A pin
  pinMode(brake1, OUTPUT); //Initiates Brake Channel A pin

  //Setup Motor2
  pinMode(dir2, OUTPUT); //Initiates Motor Channel B pin
  pinMode(brake2, OUTPUT);  //Initiates Brake Channel B pin
}

void myRobot::move(const String& Direction,double speed)
{
	int analogW = speed*scaling_factor; // analgoue value to write for PWM 
 
  
	if (Direction.equalsIgnoreCase("forward"))
	{
    m_dir = true;
		move_F(analogW); // move forward
	}
	
	else if (Direction.equalsIgnoreCase("reverse"))
	{
    m_dir = false;
		move_B(analogW); // move backward 
	}
		
}

void myRobot::move_F(int analogW)	// move forward 
{
	//Motor A 
	digitalWrite(dir1, HIGH); //Establishes direction of Channel A
	digitalWrite(brake1, LOW);   //Disengage the Brake for Channel A
	analogWrite(pwm1, analogW);   //Spins the motor on Channel A 

	//Motor B 
	digitalWrite(dir2, LOW);  //Establishes direction of Channel B
	digitalWrite(brake2, LOW);   //Disengage the Brake for Channel B
	analogWrite(pwm2, analogW);    //Spins the motor on Channel B 
	
}

void myRobot::move_B(int analogW) // move backward
{	
	//Motor A 
	digitalWrite(dir1, LOW); //Establishes direction of Channel A
	digitalWrite(brake1, LOW);   //Disengage the Brake for Channel A
	analogWrite(pwm1, analogW);   //Spins the motor on Channel A 

	//Motor B 
	digitalWrite(dir2, HIGH);  //Establishes  direction of Channel B
	digitalWrite(brake2, LOW);   //Disengage the Brake for Channel B
	analogWrite(pwm2, analogW);    //Spins the motor on Channel B 
}

void myRobot::moveM1(int analogW,bool dir)
{
  if (dir) // rotate 1 direction
  {
    digitalWrite(dir1, HIGH); //Establishes direction of Channel A
    digitalWrite(brake1, LOW);   //Disengage the Brake for Channel A
    analogWrite(pwm1, analogW);   //Spins the motor on Channel A 
  }
  else if(!dir) // rotate opposite direction 
  {
    digitalWrite(dir1,LOW); //Establishes direction of Channel A
    digitalWrite(brake1, LOW);   //Disengage the Brake for Channel A
    analogWrite(pwm1, analogW);   //Spins the motor on Channel A 
  }
}

void myRobot::moveM2(int analogW,bool dir)
{
  if (dir) // rotate 1 direction
  {
    //Motor B 
   digitalWrite(dir2, HIGH);  //Establishes  direction of Channel B
   digitalWrite(brake2, LOW);   //Disengage the Brake for Channel B
   analogWrite(pwm2, analogW);    //Spins the motor on Channel B 
  }
  else if(!dir) // rotate opposite direction 
  {
    //Motor B 
   digitalWrite(dir2, LOW);  //Establishes  direction of Channel B
   digitalWrite(brake2, LOW);   //Disengage the Brake for Channel B
   analogWrite(pwm2, analogW);    //Spins the motor on Channel B 
  }
}

void myRobot::moveSelect(int num,int analogue,bool dir)
{
  if (!num)
  {
    moveM2(analogue,dir);
  }
  else if (num==1)
  {
    moveM1(analogue,dir);
  }
}


void myRobot::brake() // stop the robot 
{
	digitalWrite(brake1, HIGH);  //Engage the Brake for Channel A
	digitalWrite(brake2, HIGH);  //Engage the Brake for Channel B
}

//--------------------------------------------------------ANGULAR FREQUENCY ENCODER FUNCTION
void encoder_init()
{
 cli(); // stop interrupts
 //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 4hz increments
  OCR1A = 3905;// = (16*10^6) / (4*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 

 sei(); // enable interrupts 
}
/*
ISR(TIMER1_COMPA_vect)//timer1 interrupt 4Hz, PUT THIS INSIDE MAIN
{
  encoder1.show();
  encoder2.show();
}
*/

//-------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------PID CONTROLLER
Controller::Controller(int number) // inistialise encoder pins 
{
    if (!number)
    {
      lpf = LowPass<2>(1.0,10.0,true);  
      encoder1_init();
      encoderMeasure = 1238;
      m_num = number;
    }
    else if (number == 1)
    {
      lpf = LowPass<2>(1,10.0,true);
      encoder2_init();
      encoderMeasure = 1228;
      m_num = number;
    }
}

void Controller::setCommand(bool dir,float lin_speed) // user input for direction and distance
{
  m_dir = dir;
  
  if (m_dir)
  {
  target = -1 * (lin_speed * scale)/radius;
  }
  else if (!m_dir)
  {
   target = (lin_speed * scale) / radius;
  }
}

void Controller::setControl(float kp,float kd,float ki) // change the PID settings 
{
  this-> kp = kp;
  this-> kd = kd;
  this-> ki = ki;
}


volatile long count[2]={0,0};

template <int j>
void encoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_A[j]) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_B[j]) == LOW) {
      count[j] = count[j] + 1;         // CW
    }
    else {
      count[j] = count[j] - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_B[j]) == HIGH) {
      count[j] = count[j] + 1;          // CW
    }
    else {
      count[j] = count[j] - 1;          // CCW
    }
  }
}

template<int j>
void encoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_B[j]) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_A[j]) == HIGH) {
      count[j] = count[j] + 1;         // CW
    }
    else {
      count[j] = count[j] - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_A[j]) == LOW) {
      count[j] = count[j] + 1;          // CW
    }
    else {
      count[j] = count[j] - 1;          // CCW
    }
  }
}


void encoder1_init()
{
  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(INTERRUPT1, encoderA<0>, CHANGE);
  // encoder pin on interrupt 5 (pin 18)
  attachInterrupt(INTERRUPT2, encoderB<0>, CHANGE);
}

void encoder2_init()
{
  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);
  // encoder pin on interrupt 0 (pin 19)
  attachInterrupt(INTERRUPT3, encoderA<1>, CHANGE);
  // encoder pin on interrupt 5 (pin 20)
  attachInterrupt(INTERRUPT4, encoderB<1>, CHANGE);
}

void Controller::Compute()
{
   ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    pos = count[m_num];
  }

   
  long currT = micros();

  float deltaT = (float)(currT-prevT)/1.0e6;
  float velocity = (pos - posPrev)/deltaT; // compute velocity 

   posPrev= pos;
   prevT = currT;
	
   v1Filt = lpf.filt(velocity);// store filtered velocity 
   
   int e = target - v1Filt;

   float dedt = (e-eprev)/(deltaT);
   eintegral = eintegral + e*deltaT;

   float u = kp*e+ kd*dedt + ki*eintegral;

   float pwr = fabs(u);// absolute value 
   if (pwr>255)
   {
     pwr = 255;
   }

    bool newdir= DEFAULTED_DIR[m_num];
    
    if (u<0)
    {
     newdir= (!newdir);
    }

   moveSelect(m_num,pwr,newdir); // control 1 motor 

   eprev = e;

   Serial.print("counts/s:\t");
   Serial.print(target);
   Serial.print("\t");
   Serial.print(v1Filt);
   Serial.println();   
}

void Controller::setAll(float tar,bool dir,float kp,float kd,float ki)
{
  
  if (dir)
  {
	tar = -1* tar * (encoderCounts/(2*pi*radius)); // convert m/s to encoder_counts/s
  }
  else if(!dir)
  {
	tar = tar * (encoderCounts/(2*pi*radius)); // convert m/s to encoder_counts/s
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    pos = count[m_num];
  }
  
  long currT = micros();

  float deltaT = (float)(currT-prevT)/1.0e6;
  float velocity = (pos - posPrev)/deltaT; // compute velocity 

  posPrev= pos;
  prevT = currT;

     // Low-pass filter (25 Hz cutoff)
//   v1Filt = 0.854*v1Filt + 0.0728*velocity + 0.0728*v1Prev;
//   v1Prev = velocity;

   v1Filt = lpf.filt(velocity);

   int e = tar - v1Filt;

   float dedt = (e-eprev)/(deltaT);
   eintegral = eintegral + e*deltaT;

   float u = kp*e+ kd*dedt + ki*eintegral;

   float pwr = fabs(u);// absolute value

   if (pwr>255)
   {
     pwr = 255;
   }

    bool newdir= DEFAULTED_DIR[m_num];
    
    if (u<0)
    {
     newdir= (!newdir);
    }

   moveSelect(m_num,pwr,newdir); // control 1 motor 

   eprev = e;

   Serial.print("counts/s:\t");
   Serial.print(tar);
   Serial.print("\t");
   Serial.println(v1Filt);

}
