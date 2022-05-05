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

1- Performing basic opertations on the robot[1]. Find files: Motor Shield -> OOP Design -> "myRobot.h" + "myRobot.cpp"

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
  	robot.brake(); // brake for 3 seconds
  	delay(3000);
  	robot.move("reverse",100); // reverse direction at 100%
        delay(3000); // for 3 seconds 
}
```

2- Controlling the Relays. Find files: Motor Shield -> OOP Design -> "Relay.h" + "Relay.cpp"

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

3- Implementing the PID Controller[2].  Find files: Motor Shield -> PID Speed Atomic -> "Control.hpp" + "Control.cpp". The tuned values for the PID are found in a .txt file "Ideal Tuning Values.txt" this is designed via George Ellis tuning method[3]. An alternate Arduino sketch with a more optimized PID function can be fouund 
in Motor Shield -> PID with Encoder. 

```cpp
#include "Control.hpp"

myRobot robot; // initialise all pins
Controller pid0(0); // create PID object for Motor 0
Controller pid1(1);// create PID object for Motor 1


void setup()
{
  Serial.begin(9600);
  pid0.setCommand(false,0.75); // set direction and speed(m/s) for PID0
  pid0.setControl(0.275,0.0850,0.55);// set Kp,Kd and Ki

  pid1.setCommand(false,0.75); // set direction and speed(m/s) for PID1
  pid1.setControl(0.30,0.0750,0.75);// set Kp,Kd and Ki
}

void loop()
{
   pid0.Compute();// start computing and driving PID signal 
   pid1.Compute();
}
        
```

# References

1. 

GitHub. 2022. Arduino-Motor-Shield-29250/two-motor.ino at master · Luen/Arduino-Motor-Shield-29250. [online] Available at: <https://github.com/Luen/Arduino-Motor-Shield-29250/blob/master/examples/two-motor.ino> [Accessed 25 March 2022].

2.

Res C. How to control multiple DC motors with encoders [Internet]. Youtube; 2021 [cited 2022 Mar 25]. Available from: https://www.youtube.com/watch?v=3ozgxPi_tl0

GitHub. 2022. GitHub - curiores/ArduinoTutorials. [online] Available at: <https://github.com/curiores/ArduinoTutorials> [Accessed 25 March 2022].

3.

Ellis G. StackPath [Internet]. www.machinedesign.com. 2000. Available from: https://www.machinedesign.com/motors-drives/article/21834627/twenty-minute-tuneup

# Encoder Datasheet

https://www.ni.com/pdf/dspdf/en/ds-217
