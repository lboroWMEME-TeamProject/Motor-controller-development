#include <PID_v1.h>
#define encoder0PinA  2
#define encoder0PinB  18
#define encoder1PinA  19
#define encoder1PinB  20

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

double Pk1=1;
double Ik1=0;
double Dk1=0.01;
double Setpoint1,Input1,Output1,Output1a;

PID PID1(&Input1,&Output1,&Setpoint1,Pk1,Ik1,Dk1,DIRECT);


double Pk2=1;
double Ik2=0;
double Dk2=0.01;
double Setpoint2,Input2,Output2,Output2a;

PID PID2(&Input2,&Output2,&Setpoint2,Pk2,Ik2,Dk2,DIRECT);


float demand1=1238;// encoder0Pos mean for 1m 
float demand2=1228;// encoder1Pos mean for 1m



int current;
int prev;

void setup() 
{
  prev= millis();
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
   pinMode(19, INPUT);
  pinMode(20, INPUT);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(5, doEncoderB, CHANGE);
    // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(4, doEncoderC, CHANGE);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(3, doEncoderD, CHANGE);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-100,100);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);
  PID2.SetMode(-100,100);
  PID2.SetSampleTime(10);  

  Serial.begin (9600);
}

void loop(){
  current = millis();
  if (current - prev >= 10)
  {
  prev=current; 
  Serial.print(encoder0Pos);
  Serial.print(" , ");
  Serial.println(encoder1Pos);
  }

  Setpoint1= demand1;
  Setpoint2= demand2;

  Input1= encoder0Pos;
  PID1.Compute();

  Input2 = encoder1Pos;
  PID2.Compute();

  if ((Output1 >0)&&(Output2>0))
  {
     Output1a= abs(Output1);
     Output2a= abs(Output2);

    
     
  }
  
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 // Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}


void doEncoderC() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == HIGH) {
      encoder1Pos = encoder1Pos + 1;          // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }
//  Serial.println (encoder1Pos, DEC);
  // use for debugging - remember to comment out
}

void doEncoderD() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinA) == LOW) {
      encoder1Pos = encoder1Pos + 1;          // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }
}
