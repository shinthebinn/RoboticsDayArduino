//create a motor group class to keep the left motors together and the right motors together
//the car has 4 DC motors, but we want to treat them like a tank with a left and right drive
#ifndef HEADER_MOTOR
  #define HEADER_MOTOR
class Motor
{
  private:
  int m_enablePin;
  int m_forwardPin;
  int m_backwardPin;

  public : 
   Motor (int enablePin, int forwardPin, int backwardPin);
  
  //Drive the motor forward or backward at a speed
  void Drive(int Speed);
  
  //stop the motor
  void Stop();
};
#endif
