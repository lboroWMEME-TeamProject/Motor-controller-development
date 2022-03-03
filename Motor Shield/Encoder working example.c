// Motor encoder output pulse per rotation (change as required)
#include <Encoder.h>
#define ENC_COUNT_REV 100
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
// Pulse count from encoder
volatile long encoderValue = 0;
 
// One-second interval for measurements
int interval = 1000;
// Variable for RPM measuerment
int rpm = 0;
// Variable for PWM motor speed output
int motorPwm = 255;

void updateEncoder();


Encoder myEnc(5,6);
long oldPosition  = -999;

void setup() {
   // Setup Serial Monitor
  Serial.begin(9600); 
  pinMode(5, INPUT_PULLUP); // setup input PWM for pin 5 encoder 
 
  
  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel A pin


 
  
}

void loop(){

  //Motor A forward @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 255);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(13, LOW);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 255);    //Spins the motor on Channel B at half speed


  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
  
}
