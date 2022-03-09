#ifndef myRobot_h
#define myRobot_h

#include <PID_v1.h>

#define ENCODER1_A 2
#define ENCODER1_B 18
#define ENCODER2_A 19
#define ENCODER2_B 20
#define RELAY1 7
#define RELAY2 10
#define RELAY3 14



class myRobot
{
	public:
	
		myRobot(); // constructor
		
		void move(const String&,double);
		void move_F(int);
		void move_B(int);
		void brake();
    bool getDIR(){return m_dir;}
		
		class Encoder // nested class for encoder init
		{
			public:
		
				Encoder(int,int); // constructor, encoder init 
				void show();
				//void enableB(); // will be added in the future 
				
			private:
				int channelA; // encoder1 pin1
				int channelB;// encoder1 pin2	 
				const int interval = 1000; // One-second interval for measurements
				const int encoder_pulses = 100; // found in the datasheet
				long previousMillis{};// Counters for milliseconds during interval
				long currentMillis{}; 
				double rpm[2]={0,0};// Variable for RPM measuerment
				static void updateEncoder();
        static void updateEncoder2();
        static volatile long encoderValue[2]; // Pulse count from encoder
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
  bool m_dir;// direction of the robot 
		
};

class Relay
{
  public:
  Relay(int,int,int);
  void On();
  void Off();
  
  private:
  int pins[3];
};

class Control
{
  public:
  Control(myRobot*,double);
  void enable();
  
  private:
  myRobot* Rptr;
  double Setpoint, Input, Output;//Define Variables we'll be connecting to
  double Kp=2, Ki=5, Kd=1;//Specify the links and initial tuning parameters
  PID* pidPtr;
};


/*
struct Timer_vars
{
	static void updateEncoder();// function for the interrupt 
//	static void updateEncoder2();
//	static void updateEncoder3();
//	static void updateEncoder4();
	
	static volatile long encoderValue[4]; // Pulse count from encoder
	static long previousMillis[4];// Counters for milliseconds during interval
	static long currentMillis[4]; 
	static double rpm[4];// Variable for RPM measuerment
};
*/
#endif
