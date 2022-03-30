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

const bool DEFAULTED_DIR[] = {true,false};

#define INTERRUPT1 0 // pin 2
#define INTERRUPT2 5 // pin 18
#define INTERRUPT3 4 // pin 19
#define INTERRUPT4 3 // pin 20

template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public: 
    LowPass()= default; 
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float c1 = 2*sqrt(2)/alpha;
        float c2 = 4/(alpha*alpha);
        float denom = 1.0+c1+c2;
        b[0] = 1.0/denom;
        b[1] = 2.0/denom;
        b[2] = b[0];
        a[0] = -(2.0-2.0*c2)/denom;
        a[1] = -(1.0-c1+c2)/(1.0+c1+c2);      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
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
  long posPrev = 0;
  long pos =0;
  float v1Filt = 0;
  float v1Prev = 0; 
  LowPass<2> lpf;
};


#endif
