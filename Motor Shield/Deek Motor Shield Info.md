The code requires pins for PWM, brake and direction change. 
Digit signal is required to change the direction.
Analogue signal is required for PWM. 

  //Motor A forward @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 255);   //Spins the motor on Channel A at full speed
  
 PWM pin is 3, DIR pin is 12, brake pin is 9. PWN Analogue write is 0-255, where 255 equates to 100% duty cycle. 
//----------------------------------------------------------

  //Motor B backward @ full speed
  digitalWrite(13, LOW);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 255);    //Spins the motor on Channel B

  PWM pin is 11, DIR pin is 13, brake pin is 8. PWN Analogue write is 0-255, where 255 equates to 100% duty cycle. 
  
  delay(3000);

  
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A(motor1)
  digitalWrite(8, HIGH);  //Engage the Brake for Channel B(motor2)

//-------------------------------------------------------------
