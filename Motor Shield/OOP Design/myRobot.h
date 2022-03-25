#ifndef myRobot_h
#define myRobot_h

//#include <PID_v1.h>

#define ENCODER1_A 2
#define ENCODER1_B 18
#define ENCODER2_A 19
#define ENCODER2_B 20
#define RELAY1 7
#define RELAY2 10
#define RELAY3 14

const int ENCODER_A[] = {ENCODER1_A,ENCODER2_A};
const int ENCODER_B[] = {ENCODER1_B,ENCODER2_B};



#define INTERRUPT1 0 // pin 2
#define INTERRUPT2 5 // pin 18
#define INTERRUPT3 4 // pin 19
#define INTERRUPT4 3 // pin 20


void interrupt_init(); // initialise encoder interrupt

void encoderPins1_init();
void encoderPins2_init();

template <int j>
void updateEncoderA();

template <int j>
void updateEncoderB();

class Relay
{
  public:
  Relay(int,int,int);
  void On();
  void Off();
  
  private:
  int pins[3];
};


class myRobot
{
	public:
	
		myRobot(); // constructor
		
		void move(const String&,double);
		void move_F(int);
		void move_B(int);
    void moveM1(int,bool);
    void moveM2(int,bool);
    void moveSelect(int,int,bool);
		void brake();
    bool getDIR(){return m_dir;}
		
		class Encoder // nested class for encoder init
		{
			public:
		
				Encoder(int); // constructor, encoder init 
				void show();
				//void enableB(); // will be added in the future 
				
			private:
        double rpm{};
				const int encoder_pulses = 400; // found in the datasheet
        int m_num;
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

class EncoderCounter
{
  public:
  EncoderCounter();

  void showCount();

  long getCountA(){return encoder0Pos;}
  long getCountB(){return encoder1Pos;}
  
  static void doEncoderA();
  static void doEncoderB();
  static void doEncoderC();
  static void doEncoderD();
  
  private:
  static volatile long encoder0Pos;
  static volatile long encoder1Pos;
 
};





#endif
