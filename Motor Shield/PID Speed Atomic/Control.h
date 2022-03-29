#ifndef Control_h
#define Control_h

#define ENCODER1_A 2
#define ENCODER1_B 18
#define ENCODER2_A 19
#define ENCODER2_B 20
#define RELAY1 7
#define RELAY2 10
#define RELAY3 14

const double pi {3.1415926535};

const int ENCODER_A[] = {ENCODER1_A,ENCODER2_A};
const int ENCODER_B[] = {ENCODER1_B,ENCODER2_B};

const bool DEFAULTED_DIR[] = {false,true};

#define INTERRUPT1 0 // pin 2
#define INTERRUPT2 5 // pin 18
#define INTERRUPT3 4 // pin 19
#define INTERRUPT4 3 // pin 20

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

void encoder_init();
void encoder1_init();
void encoder2_init();

class Controller: public myRobot
{
  public:
  
  Controller(int);
  void setCommand(bool,float);
  void setControl(float,float,float);
  void setAll(float,bool,float,float,float);
  void Compute();
  
  private:
  int m_num;
  int m_encoderPins[2];
  bool m_dir;
  float kp = 5;
  float kd = 0.5;
  float ki = 0;
  int target;
  int encoderMeasure;
  long prevT =0;
  float eprev = 0;
  float eintegral = 0;
  const int encoderCounts{400}; // encoder clicks in 1 rev
  const float radius{0.05}; // radius of 5 cm  
  const float scale{63.66198}; // scaling factor to convert m/s to clicks/s, 200/pi
  int posPrev = 0;
  long pos =0;
  float v1Filt = 0;
  float v1Prev = 0; 
};


#endif
