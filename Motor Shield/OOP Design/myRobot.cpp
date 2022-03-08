#include "myRobot.h"

volatile long myRobot::Encoder::encoderValue[]={0,0};

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
myRobot::Encoder::Encoder(int a, int b)
: channelA{a},channelB{b}
{
	Serial.begin(9600);
  
	pinMode(channelA, INPUT_PULLUP); // setup input PWM for encoder pin
	pinMode(channelB, INPUT_PULLUP); // setup input PWM for encoder pin  
	
	attachInterrupt(digitalPinToInterrupt(channelA),updateEncoder, RISING);
	attachInterrupt(digitalPinToInterrupt(channelB), updateEncoder, RISING);

	
	previousMillis[0] = millis(); // start timer
  previousMillis[1] = millis(); // start timer  
}


void myRobot::Encoder::show()
{
  currentMillis[0] = millis();
  currentMillis[1] = millis();
  
  if (currentMillis[0] - previousMillis[0] > interval)
  {
    previousMillis[0] = currentMillis[0];
    previousMillis[1] = currentMillis[1];
 
    // Calculate RPM
    rpm[0] = (encoderValue[0] * 60  / static_cast<double>(encoder_pulses)); // RPM calculation
    rpm[1] = (encoderValue[1] * 60  / static_cast<double>(encoder_pulses)); // RPM calculation  

    Serial.print("Encoder(RPM): ");
    Serial.print(rpm[0]);
    Serial.print("\t");
    Serial.println(rpm[1]);
    
    encoderValue[0] = 0;
    encoderValue[1] = 0;
  }
  
}

void myRobot::Encoder::updateEncoder()
{
  encoderValue[0]++; // Increment value for each pulse from encoder
}

void myRobot::Encoder::updateEncoder2()
{
  encoderValue[1]++;// Increment value for each pulse from encoder
}
