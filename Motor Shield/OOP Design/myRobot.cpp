#include "Arduino.h"
#include "myRobot.h"

volatile long EncoderCounter::encoder0Pos=0;
volatile long EncoderCounter::encoder1Pos=0;

volatile long counter[2] = {0,0};

EncoderCounter::EncoderCounter()
{
  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT);
  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(INTERRUPT1, doEncoderA, CHANGE);
  // encoder pin on interrupt 5 (pin 18)
  attachInterrupt(INTERRUPT2, doEncoderB, CHANGE);
   // encoder pin on interrupt 0 (pin 19)
  attachInterrupt(INTERRUPT3, doEncoderC, CHANGE);
  // encoder pin on interrupt 5 (pin 20)
  attachInterrupt(INTERRUPT4, doEncoderD, CHANGE);
}

void EncoderCounter::doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER1_A) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER1_B) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER1_B) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
//  Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

void EncoderCounter::doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER1_B) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER1_A) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER1_A) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

void EncoderCounter::doEncoderC() {
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER2_A) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER2_B) == LOW) {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER1_B) == HIGH) {
      encoder1Pos = encoder1Pos + 1;          // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }
//  Serial.println (encoder1Pos, DEC);
  // use for debugging - remember to comment out
}



void EncoderCounter::doEncoderD() {
  // look for a low-to-high on channel D
  if (digitalRead(ENCODER2_B) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER2_A) == HIGH) {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER2_A) == LOW) {
      encoder1Pos = encoder1Pos + 1;          // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }
}

void EncoderCounter::showCount()
{

  Serial.print(encoder0Pos);
  Serial.print(" , ");
  Serial.println(encoder1Pos);
}


Relay::Relay(int a,int b,int c)
{
  pins[0]=a;
  pins[1]=b;
  pins[2]=c;


  pinMode(pins[0], OUTPUT); 
  pinMode(pins[1], OUTPUT); 
  pinMode(pins[2], OUTPUT); 
}

void Relay::On() // LOW= HIGH due to incorrect wiring 
{
  digitalWrite(pins[0], LOW); 
  digitalWrite(pins[1], LOW);  
  digitalWrite(pins[2], LOW); 

}

void Relay::Off()
{
  digitalWrite(pins[0], HIGH); 
  digitalWrite(pins[1], HIGH);  
  digitalWrite(pins[2], HIGH); 
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


//------------------------------------------------------------------------------ENCODER DEFINITION

void encoderPins1_init()
{
  pinMode(ENCODER_A[0], INPUT_PULLUP); // setup input PWM for encoder pin
  pinMode(ENCODER_B[0], INPUT_PULLUP); // setup input PWM for encoder pin  
  
  attachInterrupt(INTERRUPT1,updateEncoderA<0>, CHANGE);
  attachInterrupt(INTERRUPT2, updateEncoderB<0>, CHANGE);
}

void encoderPins2_init()
{
  pinMode(ENCODER_A[1], INPUT_PULLUP); // setup input PWM for encoder pin
  pinMode(ENCODER_B[1], INPUT_PULLUP); // setup input PWM for encoder pin  
  
  attachInterrupt(INTERRUPT3,updateEncoderA<1>, CHANGE);
  attachInterrupt(INTERRUPT4, updateEncoderB<1>, CHANGE);
}


template <int j>
void updateEncoderA()
{
    // look for a low-to-high on channel A
  if (digitalRead(ENCODER_A[j]) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_B[j]) == LOW) {
      counter[j] = counter[j] + 1;         // CW
    }
    else {
      counter[j] = counter[j] - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_B[j]) == HIGH) {
      counter[j] = counter[j] + 1;          // CW
    }
    else {
      counter[j] = counter[j] - 1;          // CCW
    }
  }
}


template <int j>
void updateEncoderB()
{
   // look for a low-to-high on channel B
  if (digitalRead(ENCODER_B[j]) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_A[j]) == HIGH) {
      counter[j] = counter[j] + 1;         // CW
    }
    else {
      counter[j] = counter[j] - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_A[j]) == LOW) {
      counter[j] = counter[j] + 1;          // CW
    }
    else {
      counter[j] = counter[j] - 1;          // CCW
    }
  }
}

myRobot::Encoder::Encoder(int num)
{
  if (!num)
  {
    encoderPins1_init();
    m_num = num;
  }
  else if (num == 1)
  {
    encoderPins2_init();
    m_num = num;
  }
}


void myRobot::Encoder::show()
{
      // Calculate RPM
    rpm = (counter[m_num] * 60*4 / (static_cast<double>(encoder_pulses))); // RPM calculation

    Serial.print("Encoder(RPM): ");
    Serial.println(rpm);

    
    counter[m_num] = 0;
}



void interrupt_init()
{
 Serial.begin(9600);
 
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
ISR(TIMER1_COMPA_vect)//timer1 interrupt 4Hz
{
  encoder1.show();
  encoder2.show();
}
*/
