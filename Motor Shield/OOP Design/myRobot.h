#ifndef myRobot_h
#define myRobot_h

#include "Arduino.h"

void updateEncoder();// global function for interrupt

class myRobot
{
	public:
	
	myRobot(); // constructor
	
	void move(const String&,double);
	void move_F(int);
	void move_B(int);
	void brake();
	
	class Encoder // nested class for encoder init
	{
		public:
	
		Encoder(); // constructor, encoder init 
		
		void enableA();
		//void enableB(); // will be added in the future 
			
		private:
		// Interrupt pins for the Arduino mega 
		const int channel_1A = 2; // encoder1 pin1
		const int channel_1B = 18;// encoder1 pin2
		
		const int channel_2A= 19;// encoder2 pin1
		const int channel_2B=20;// encoder2 pin2	 
		// One-second interval for measurements
		const int interval = 1000; // ms in s
		
		const int encoder_pulses = 100; // found in the datasheet
	};
	
	private:
	
	const double scaling_factor = 2.55;// 8 bit input for PWM 
	
	//-------------------Pin numbers for the Deek Motor Sheild
	//Motor1
	const int pwm1 = 3;// PWM
	const int dir1 = 12;// Direction
	const int brake1= 9;// Brake 
	//Motor2
	const int pwm2 = 11;
	const int dir2 = 13;
	const int brake2= 8;
	
};



#endif
