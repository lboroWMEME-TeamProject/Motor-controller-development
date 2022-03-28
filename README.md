# PID Controller Design and Implementation


![image](https://user-images.githubusercontent.com/92602684/160253302-a9bdf04f-4aa9-4272-a4fc-e1f518835d3a.png)

<p align='center'>
    <a href ="https://github.com/EoinBrennan2000">
        <img src="https://img.shields.io/badge/Author-@EoinBrennan2000-blueviolet.svg?style=flat" /></a>
    <a href ="https://github.com/areebTP">
        <img src="https://img.shields.io/badge/Author-@areebTP-blue.svg?style=flat"/></a>
    <a href ="https://github.com/AleksNowak">
        <img src="https://img.shields.io/badge/Author-@AleksNowak-green.svg?style=flat"/></a>
    <a href ="https://github.com/LC-11115">
        <img src="https://img.shields.io/badge/Author-@LC--11115-yellow.svg?style=flat"/></a>
    <a href ="https://github.com/ECorpAdmin">
        <img src="https://img.shields.io/badge/Author-@ECorpAdmin-red.svg?style=flat"/></a>



# Description 

This is a subsystem of the COVID cleaning robot which comprises of the object-oritented design in C++. The motor controller subsytem consits of header/source files required to move the robot and read from qudrature encoder channels. The PID controller is an integral part of our robot as it forms the basis of the final navigation system we plan to implement.  

# Installation & Usage 

1- Performing basic opertations on the robot[1]

```cpp
 #include "myRobot.h" // include this file for pins initialisation
// #include "Control.h" // OR include this header 

myRobot robot; // create Robot object


void setup()
{}

void loop()
{
 	robot.move("forward",100); // move the robot forward at 100%
  	delay(3000); // wait for 3 seconds
  	robot.brake(); // brake
  	delay(3000);
  	robot.move("reverse",100); // reverse direction at 100%
  
}
```

2- Controlling the Relays 

```cpp
#include "Relay.h" 

Relay relay(RELAY1,RELAY2,RELAY3);// replace these with global definitions or any other pins

void setup()
{}

void loop()
{
//  relay.On(); // turn ON ALL pins
//  delay(100); // on for 100ms
//  relay.Off();// turn OFF ALL pins
// delay(100);// off for 100ms
  
  relay.setPin(1,"off"); // turn off pin # 1 = PIN10
//relay.setPin(0,"ON"); // turn on pin # 0 = PIN7
//  relay.setPin(2,"off"); // turn off pin # 2 = PIN14
}
```

3- Implementing the PID Controller[2]

```cpp
#include "Control.h"

myRobot robot; // initialise all pins
Controller pid1(0); // initialise PID for Motor1
Controller pid2(1);// initialise PID for Motor2

void setup()
{
  Serial.begin(9600);
  pid1.setCommand(true,1.5); // arg1 = direction, arg2= distance in meters
  pid1.setControl(5,0.5,0); // kp = 0.5, kd = 0.5, ki = 0

  pid2.setCommand(true,1.5); 
  pid2.setControl(3,0.5,0);  
}

void loop()
{
  pid1.Compute(); // start the PID controll MUST be placed inside  a while loop or fast interrupt
  pid2.Compute();
}
```
4- Measuring angular frequency(RPM) of the motors using the encoders[3]

```cpp
#include "myRobot.h" // include the robot file 

myRobot robot; // initialise all pins
myRobot::Encoder encoder1(0); // insitialise the encoder1 pins
myRobot::Encoder encoder2(1); // insitialise the encoder2 pins

void setup()
{
  interrupt_init();// enable the encoder interrupts 
}

void loop()
{
 //  robot.move("forward",100);
}

ISR(TIMER1_COMPA_vect)//timer1 interrupt 4Hz
{
  encoder1.show(); // display RPM from both encoders 
  encoder2.show();
}

```

# References

1. 

GitHub. 2022. Arduino-Motor-Shield-29250/two-motor.ino at master · Luen/Arduino-Motor-Shield-29250. [online] Available at: <https://github.com/Luen/Arduino-Motor-Shield-29250/blob/master/examples/two-motor.ino> [Accessed 25 March 2022].

2.

Res C. How to control multiple DC motors with encoders [Internet]. Youtube; 2021 [cited 2022 Mar 25]. Available from: https://www.youtube.com/watch?v=3ozgxPi_tl0

GitHub. 2022. GitHub - curiores/ArduinoTutorials. [online] Available at: <https://github.com/curiores/ArduinoTutorials> [Accessed 25 March 2022].

3.

DroneBot Workshop. Using rotary encoders with arduino [Internet]. Youtube; 2019 [cited 2022 Mar 25]. Available from: https://www.youtube.com/watch?v=V1txmR8GXzE

# Encoder Datasheet

https://www.ni.com/pdf/dspdf/en/ds-217
