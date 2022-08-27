//create a motor group class to keep the left motors together and the right motors together
//the car has 4 DC motors, but we want to treat them like a tank with a left and right drive
#include <Arduino.h>
#include "Motor.h"

Motor::Motor (int enablePin, int forwardPin, int backwardPin)
{
    m_enablePin=enablePin;
    m_forwardPin=forwardPin;
    m_backwardPin=backwardPin;
    Stop();
    
};

//Drive the motor forward or backward at a speed
void Motor::Drive(int Speed)
{
    if(Speed>0)
    {
      digitalWrite(m_backwardPin, LOW);
      digitalWrite(m_forwardPin, HIGH);
      analogWrite(m_enablePin,Speed);
 
      Serial.print("Drive: Enabled pin: ");
      Serial.print(m_enablePin);
      Serial.print(" Drive: driving forwards at ");
      Serial.println(Speed);
    }
    else if(Speed<0)
    {
      
      digitalWrite(m_forwardPin, LOW);
      digitalWrite(m_backwardPin, HIGH);
      analogWrite(m_enablePin, abs(Speed));
      
      Serial.print(m_enablePin);
      Serial.print("Drive: driving backwards at ");
      Serial.println(abs(Speed));
    }
    else if(Speed==0)
    {
      Stop();
    }
};

//stop the motor
void Motor::Stop()
{
    Serial.println("Drive:Stop - Stop Motor");
      
      digitalWrite(m_forwardPin, LOW);
      digitalWrite(m_backwardPin, LOW);
      analogWrite(m_enablePin, 0);
      
};
