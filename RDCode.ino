//
// Example drive code for the Robots Day Gearheads challenge car.
//  follow a line until a T is detected & then turn 90 degrees
//
#include <Arduino.h>
#include "Motor.h"

//Power - 9v battery to arduino barrel jack
//AA batter powers the motors 1.5Vx4=6V

/*
 * Global variables & constant pin definitions
 *   Note: global variables are used to keep
 *   values between loop() iterations
 */

//5v to bb 5v rail
//gnd to bb gnd rail

//Motor controller H-Bridge Pins for Left & Right Motors
#define MEnRight 6   //Motor Right Enable
#define MEnLeft 5
#define MRForw 7
#define MRBack 8
#define MLForw 10
#define MLBack 9

//Proximity sensors
#define LeftIRpin A1
#define RightIRpin A2

//Encoder Pins
//vcc - goes to 5v bb rail
//gnd - goes to gnd bb rail
#define LeftEncoder 2  //out pin on left photo sensor to arduino 2
#define RightEncoder 3 //out pin on right photo sensor to arduino 3

//number of ticks in 1 rotation for encoder
#define ENCODERMULT 20

//Wheel Circumference = pi* wheel diameter. TicksPerInch = Wheel Circumf/slots
#define TicksPerInch 2.4

//chassis circumf = circumference of the circle if the car turns in place with one
//motor move forward & one moving backward = pi * Wheel base (or chassis width)
//Note: the wheels are fat & not flat, the wheel base is really measured from near
// the middle of each tire - you should measure your car, for better results.
#define ChassisCircumf 18.0

//encoder tickCounting variables must be volatile - upated
// in interrupt routines
volatile uint32_t tickCountA = 0; //left
volatile uint32_t tickCountB = 0; //right


void setup() {
  // put your setup code here, to run once:
  
  //define the pin modes for the motor controller
  pinMode(MEnRight,OUTPUT);
  pinMode(MEnLeft,OUTPUT);
  pinMode(MRForw,OUTPUT);
  pinMode(MRBack,OUTPUT);
  pinMode(MLForw,OUTPUT);
  pinMode(MLBack,OUTPUT);

  //start the serial monitor for debug write statements
  Serial.begin(115200);

  //define the pin modes for the encoders & attach interrupts
  pinMode(LeftEncoder, INPUT_PULLUP);
  pinMode(RightEncoder, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(LeftEncoder), interruptA, RISING);
  attachInterrupt(digitalPinToInterrupt(RightEncoder), interruptB, RISING);
  delay(100);
  
  Serial.println("Completed Setup...");
}

//Create motor instances for the left & right motors with proper pin defs
Motor leftMotor = Motor(MEnLeft,MLForw,MLBack);
Motor rightMotor = Motor(MEnRight,MRForw,MRBack);

bool moreLine = true;
bool DoneLineFollow = false;
bool DoneDriving = false;

//loop runs over and over again until the arduino is reset
void loop() {
  
  leftMotor.Stop();
  rightMotor.Stop();
  
/*
 * dead reckoning using encoders to 
 * figure out the distance driven or turning
 */
   Serial.println("follow the line");
  //follow the line: speedToDrive, turnSpeed, minimumDelay

  if (!DoneLineFollow){
      Serial.println("follow the line");
      //follow the line: speedToDrive, turnSpeed, minimumDelay
      moreLine = irLineFollow(100, 60, 5); //return false on STOPPED
      if (!moreLine) {
        DoneLineFollow = true;
      }
  }
  else if (!DoneDriving){
        Serial.println("90 deg turn");
        turnDegrees(90,75); 
    
        DoneDriving = true;
  }
  Serial.println("End Loop()...");
}
////
//Define useful methods here
//
/*
 * Drive the motors based on a time in milliseconds
 *  DriveTime(int leftSpeed, int rightSpeed, int milliseconds)
 *   were leftSpeed & rightSpeed control the left & right motor speed &
 *   direction & Time is the time to drive in milliseconds. Motor stop when
 *   time expires
 */
 void DriveTime(int leftSpeed, int rightSpeed, int timeToDrive){
  leftMotor.Drive(leftSpeed);
  rightMotor.Drive(rightSpeed);
  delay(timeToDrive);
  leftMotor.Stop();
  rightMotor.Stop();
 }
 
 /*
  * Encoder functions
  */

//interrupts occur each time the sensor switches from no light to sensing light
//or each time it detects a gap in the wheel. There are 19 cutouts in the wheel
//so one rotation = 19 ticks
void interruptA() {
  tickCountA++;
}
//interruptB is the same as A, except for the other wheels encoder
// a tick or interrupt occurs each time the sensor sees a change hole in the
// wheel - there are 19 per rotation.
void interruptB() {
  tickCountB++;
}
//Drive the requested distance in inches, at the given speed and in the 
//given direction, where direction is either forward or backward in a 
//straight line. C = pi*Diameter (C=3.14*2.5=7.85)
//ticksPerInch = 19/7.85 = 2.4
void driveDistance(float distToGo, int speedToDrive){

  tickCountA = 0;
  tickCountB = 0;
  bool driveForward = true;
  if (distToGo < 0) {
    driveForward = false;
    distToGo = -distToGo;
  }
  uint32_t ticksNeeded = distToGo*TicksPerInch;
  if (driveForward){
    leftMotor.Drive(speedToDrive);
    rightMotor.Drive(speedToDrive);
  }
  else {
    leftMotor.Drive(-speedToDrive);
    rightMotor.Drive(-speedToDrive);
  }
  while(tickCountA < ticksNeeded){
    Serial.print("Tick Count: ");
    Serial.print(tickCountA);
    Serial.print(" TickCountB: ");
    Serial.print(tickCountB);
    Serial.print(" TicksNeeded: ");
    Serial.println(ticksNeeded);
    if (abs(tickCountA - tickCountB) >5) {
      Serial.println("WARNING!!! motors are moving at diff speeds");
    }
    delay(20);
  }
  leftMotor.Stop();
  rightMotor.Stop();
  
}
//turn in place - left and right wheels going in opposite directions at the 
// same speed for a given number of degrees. The distance is determined by the 
// circumference of the circle where the length of the wheel base (6.37") is the diameter.
// 360 degree turn has a circumference of pi*D = 3.14*6.37" = 20.0",
// so the distance to travel is the percent of the turn * full distance(circumference)
void turnDegrees(float degreesToTurn, int speedToDrive){
  tickCountA = 0;
  tickCountB = 0;
  bool turnClockwise = true;
  if (degreesToTurn < 0) {
    turnClockwise = false;
    degreesToTurn = -degreesToTurn;
  }
  float percentOfTurn = degreesToTurn/360.0;
  float distanceToTravel = percentOfTurn*ChassisCircumf;
  uint32_t ticksNeeded = distanceToTravel*TicksPerInch;
  Serial.print("Ticks needed to complete turn: ");
  Serial.println(ticksNeeded);
  if (turnClockwise){
    Serial.println("Turning Clockwise");
    leftMotor.Drive(speedToDrive);
    rightMotor.Drive(-speedToDrive);
  }
  else {
    Serial.println("Turning Counter-clockwise");
    leftMotor.Drive(-speedToDrive);
    rightMotor.Drive(speedToDrive);
  }
  while (tickCountA < ticksNeeded){
    Serial.print("TickCountA: ");
    Serial.print(tickCountA);
    Serial.print(" TickCountB: ");
    Serial.print(tickCountB);
    Serial.print(" TicksNeeded: ");
    Serial.println(ticksNeeded);
    if (abs(tickCountA - tickCountB) > 5){
      Serial.println("WARNING!!! Motors are driving at diff speeds");
    }
    delay(20);
  }
  leftMotor.Stop();
  rightMotor.Stop();
}

/*
 * line following functions
 */
//follow a black/dark line on a light surface
// The light surface will reflect back & read a
// high number, the line/dark surface will not 
// reflect back and will read a low number.
// if you see a straight dark line across the field stop.
bool irLineFollow(int speedToDrive, int turnDiff, int minDelay){
  int leftIR = analogRead(LeftIRpin);
  int rightIR = analogRead(RightIRpin);
  bool moreLine = true;
  
  Serial.print("left IR sensor: ");
  Serial.println(leftIR);
  Serial.print("Right IR sensor: ");
  Serial.println(rightIR);

  if (leftIR > 500 && rightIR < 500) {
    Serial.println("Go LEFT");
    leftMotor.Drive(speedToDrive - turnDiff);
    rightMotor.Drive(speedToDrive);
  }
  else if (leftIR <500 && rightIR > 500){
    Serial.println("Go Right");
    leftMotor.Drive(speedToDrive);
    rightMotor.Drive(speedToDrive - turnDiff);

  }
  else if (leftIR < 500 && rightIR < 500){
    Serial.println("Go Straight");
    leftMotor.Drive(speedToDrive);
    rightMotor.Drive(speedToDrive);
  }
  else if (leftIR > 500 && rightIR > 500){
    Serial.println("STOP");
      leftMotor.Stop();
      rightMotor.Stop();
       moreLine = false;
  }
  delay(minDelay);
  return moreLine;
}
