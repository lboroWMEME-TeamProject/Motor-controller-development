#include "myRobot.h"

//------------------------------------------INITIALISING THE TIMER_VARS MEMBERS
volatile long Timer_vars::encoderValue = 0; 
long Timer_vars::previousMillis = 0;
long Timer_vars::currentMillis = 0; 
double Timer_vars::rpm = 0;

void Timer_vars::updateEncoder()
{
  // Increment value for each pulse from encoder
  encoderValue++;
}
//--------------------------------------------------------------------------------

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
		move_F(analogW); // move forward
	}
	
	else if (Direction.equalsIgnoreCase("reverse"))
	{
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

void myRobot::brake() // stop the robot 
{
	digitalWrite(brake1, HIGH);  //Engage the Brake for Channel A
	digitalWrite(brake2, HIGH);  //Engage the Brake for Channel B
}


//------------------------------------------------------------------------------ENCODER DEFINITION
myRobot::Encoder::Encoder()
{
  Serial.begin(9600);
  
	pinMode(channel_1A, INPUT_PULLUP); // setup input PWM for encoder pin
	pinMode(channel_1B, INPUT_PULLUP); // setup input PWM for encoder pin  
	pinMode(channel_2A, INPUT_PULLUP); // setup input PWM for encoder pin  
	pinMode(channel_2B, INPUT_PULLUP); // setup input PWM for encoder pin  
	
	attachInterrupt(digitalPinToInterrupt(channel_1A),Timer_vars::updateEncoder, RISING);
//	attachInterrupt(digitalPinToInterrupt(channel_1B), updateEncoder, RISING);
//	attachInterrupt(digitalPinToInterrupt(channel_2A), updateEncoder, RISING);
//	attachInterrupt(digitalPinToInterrupt(channel_2B), updateEncoder, RISING);
	
	Timer_vars::previousMillis = millis(); // start timer 
}


void myRobot::Encoder::enableA()
{
	Timer_vars::currentMillis = millis();
	
	if (Timer_vars::currentMillis - Timer_vars::previousMillis > interval) 
	{
		Timer_vars::previousMillis = Timer_vars::currentMillis;
 
 
    // Calculate RPM
		Timer_vars::rpm = (Timer_vars::encoderValue * 60  / static_cast<double>(encoder_pulses)); // RPM calculation 
 
		Serial.print("Encoder1: ");
		Serial.print('\t');
		Serial.print(" ChannelA:");
		Serial.print(Timer_vars::rpm);
		Serial.print(" RPM");
    Serial.print("\tChannelB:");
    Serial.println(" RPM");
		
		Timer_vars::encoderValue = 0;
  }
}
