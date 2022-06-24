
/*
      NITRO Clubs EU - Network of IcT Robo Clubs
 
 WEB site: https://www.nitroclubs.eu 
 GitHub repositories: https://github.com/nitroclubs?tab=repositories 
 
          NITRObot Line following with obstacle avoidance v2.0
*/

///////////////////       IMPORTANT!    ///////////////////////////////////////////// 
/// In order to prolong the Life of the 18650 Battery:
/// DO NOT DISCHAREGE NITRObot's BATTERIES LOWER THAN 3.2V, WHICH IS 6.4V MESURED BY THE NITRObot's VOLTMETER (2x3.2V)
/// NEVER!!! NEVER DISCHARGE A BATTERY BELOW 2.8V, WHICH IS 5.6V (2x2.8v) MEASURED BY THE NITRObot's VOLTMETER  
///////////////////       IMPORTANT!    ///////////////////////////////////////////// 


// N O T E :
// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximately at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.

#include <Arduino.h>

#include <Servo.h> // If not installed, install Arduino librarry Servo by Michael Margolis from the librarry manager

// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximately at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.

//#include <Servo.h>

#define LEFT_FOR 9    // 
#define LEFT_BACK 5   // 
#define RIGHT_FOR 6   // 
#define RIGHT_BACK 10 // 

#define LN_SENS_PIN_RIGHTEDGE 22 // right edge sensor - Connected to D1 pin of the sensor
#define LN_SENS_PIN_RIGHT 23       // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_MIDDLE 24 // middle sensor - Connected to D3 pin of the sensor
#define LN_SENS_PIN_LEFT 25       // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFTEDGE 26 // left edge sensor - Connected to D5 pin of the sensor
#define LN_SENS_CALIB_PIN 27    // Connected to CAL pin of the sensor
#define LN_SENS_ANALOG_PIN A15  // Connected to AN pin of the sensor

const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3;
const int RgbPin = 2;
const int ServoPin = 13;
const int LedPin = 33;

const int MazeCorridorWidth = 50;

const float WallToCorridorMiddle = MazeCorridorWidth / 2;
const float SideCorridorTreshold = MazeCorridorWidth;

const float CenterLineTolerance = 2.5; // plus/minus how many cm are acceptable to consider the movement to be on the center line...
                                       // +- 1 cm from centerline is considered straight movement!!!
const float SharpTurnTreshold = 15.0;  // Measured by experiments with the robot
const int WallFollowingSide = -90;     // Set: -90 for right wall following or +90 for left wall following
                                       // we will add this value to the servo position i.e. myservo.write(90 + WallFollowingSide);
                                       //  in order to set to which side the servo should move (0 or 180 degrees)
// Servo parameters
const int FrontServoAngle = 90;
const int SideServoAngle = FrontServoAngle + WallFollowingSide; //(0 or 180 degrees)
const int FrontServoDelay = 90;
const int SideServoDelay = 90;

////////////////////////////////////////////////////////////////////
// SETINGS FOR NITRObot number one (values aquired by using the NITRObot Bluetooth callibration utility)
// Uncomment the settings for the NITRObot in use!
// Do not forget to comment the settings for the other NITRObot!
//////////////////////////////////////

// const int LeftSpeed = 100;  //да се подбере оптималната скорост на левия двигател
// const int RightSpeed = 100; //да се подбере оптималната скорост на десния двигател

// const int rightTimedTurnSpeedLeft = 150;  // Left side speed at which the 90 degree turn to be executed
// const int rightTimedTurnSpeedRight = 159; // Right side speed at which the 90 degree turn to be executed
// const int right90turnTiming = 505;     // Delay in ms for complete 90 degree turn to be executed at the given speed above
// const int right25turnTiming = 163;     //??? Delay in ms for complete 25 degree turn to be executed at the given speed above

// const int leftTimedTurnSpeedLeft = 150;  //?? Left side speed at which the 90 degree turn to be executed
// const int leftTimedTurnSpeedRight = 159; //?? Right side speed at which the 90 degree turn to be executed
// const int left90turnTiming = 550;     //?? Delay in ms for complete 90 degree turn to be executed at the given speed above
// const int left80turnTiming = 235;     // 230??250 Delay in ms for complete 80 degree turn to be executed at the given speed above

// const int forward20cmTiming = 700; //650;  // ????
// const int forward30cmTiming = 1100; //1000; // 900????

// const float FrontDistanceTreshold = 15;

// const int leadingWheelShallowTurnSpeed = 180; 
// const int leadingWheelSlightTurnSpeed = 200;
// const int leadingWheelSharpTurnSpeed = 220;
// const int leadingWheelAgresiveTurnSpeed = 255;

//////////// END SETTINGS FOR NITRObot number one //////////////////
////////////////////////////////////////////////////////////////////

//*****************************************************************

/////////////////////////////////////////////////////////////////////
// SETINGS FOR NITRObot number two (values aquired by using the NITRObot Bluetooth callibration utility)
// Uncomment the settings for the NITRObot in use!
// Do not forget to comment the settings for the other NITRObot!
//////////////////////////////////////

const int LeftSpeed = 90; // 85 + 31;  //да се подбере оптималната скорост на левия двигател
const int RightSpeed = 90; //85; //да се подбере оптималната скорост на десния двигател

const int rightTimedTurnSpeedLeft = 180;  // Left side speed at which the 90 degree turn to be executed
const int rightTimedTurnSpeedRight = 150; // Right side speed at which the 90 degree turn to be executed
const int right90turnTiming = 433;     // Delay in ms for complete 90 degree turn to be executed at the given speed above
const int right25turnTiming = 163;     //Delay in ms for complete about 25 degree turn to be executed at the given speed above

const int leftTimedTurnSpeedLeft = 180;  // Left side speed at which the 90 degree turn to be executed
const int leftTimedTurnSpeedRight = 150; // Right side speed at which the 90 degree turn to be executed
const int left90turnTiming = 370;     // Delay in ms for complete 90 degree turn to be executed at the given speed above
const int left80turnTiming = 150;     // Delay in ms for complete 80 degree turn to be executed at the given speed above

const int forward20cmTiming = 750; //650;  
const int forward30cmTiming = 900; //900; 

const float FrontDistanceTreshold = 20;

const int leadingWheelShallowTurnSpeed = 160; //180;
const int leadingWheelSlightTurnSpeed = 180; //200;
const int leadingWheelSharpTurnSpeed = 210; //220;
const int leadingWheelAgresiveTurnSpeed = 245;//230; //255;

//////////// END SETTINGS FOR NITRObot number two //////////////////
////////////////////////////////////////////////////////////////////

// float maxDistance = 130.0;
int speedLeft = LeftSpeed;
int speedRight = RightSpeed;
int currentState = 0;

int leftEdge;
int left;
int mid;
int right;
int rightEdge;

Servo servo;

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMoving();
void avoidObstacle();
float getDistance();
int getCurrentstate();

//-----------------------------------------------

void setup()
{
  pinMode(LN_SENS_PIN_RIGHTEDGE, INPUT);
  pinMode(LN_SENS_PIN_RIGHT, INPUT);
  pinMode(LN_SENS_PIN_MIDDLE, INPUT);
  pinMode(LN_SENS_PIN_LEFT, INPUT);
  pinMode(LN_SENS_PIN_LEFTEDGE, INPUT);
  // pinMode(LN_SENS_CALIB_PIN, OUTPUT);
  pinMode(LN_SENS_ANALOG_PIN, INPUT);

  pinMode(ServoPin, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LedPin, OUTPUT);

  servo.attach(ServoPin);
  servo.write(FrontServoAngle); // 90° - Set servo angle so that the ultrasound sensor points straight ahead

  Serial1.begin(9600);

  delay(1000); // Delay the start of NITRObot's movement
}

//---------------------------------------------------------

void loop()
{
  //
  leftEdge = digitalRead(LN_SENS_PIN_RIGHTEDGE);
  left = digitalRead(LN_SENS_PIN_RIGHT);
  mid = digitalRead(LN_SENS_PIN_MIDDLE);
  right = digitalRead(LN_SENS_PIN_LEFT);
  rightEdge = digitalRead(LN_SENS_PIN_LEFTEDGE);


  switch (getCurrentstate())
  {
  case 1: //  0 0 1 0 0
    speedLeft = LeftSpeed;
    speedRight = RightSpeed;
    moveForward();
    Serial1.println("1");
    break;
  case 2: //  0 1 1 0 0
    speedLeft = LeftSpeed * .6;
    speedRight = leadingWheelShallowTurnSpeed; //180
    moveForward();
    Serial1.println("2");
    break;
  case 3: //   0 1 0 0 0
    speedLeft = LeftSpeed * .4;
    speedRight = leadingWheelSlightTurnSpeed; //200;
    moveForward();
    Serial1.println("3");
    break;
  case 4: //  1 1 0 0 0
    speedLeft = LeftSpeed * .3;
    speedRight = leadingWheelSharpTurnSpeed; //220;
    Serial1.println("4");
    moveForward();
    break;
  case 5: //  0 0 1 1 0
    speedLeft = LeftSpeed * .2;
    speedRight = leadingWheelAgresiveTurnSpeed; //255;
    moveForward();
    Serial1.println("5");
    break;
  case 6: //  0 0 1 1 0
    speedLeft = leadingWheelShallowTurnSpeed; //180;
    speedRight = RightSpeed * .6;
    moveForward();
    Serial1.println("6");
    break;
  case 7: //   0 0 0 1 0
    speedLeft = leadingWheelSlightTurnSpeed; //200;
    speedRight = RightSpeed * .4;
    moveForward();
    Serial1.println("7");
    break;
  case 8: //  0 0 0 1 1
    speedLeft = leadingWheelSharpTurnSpeed; //220;
    speedRight = RightSpeed * .3;
    moveForward();
    Serial1.println("8");
    break;
  case 9: //  0 0 0 0 1
    speedLeft = leadingWheelAgresiveTurnSpeed; //255;
    speedRight = RightSpeed * .2;
    moveForward();
    Serial1.println("9");
    break;
  case 10: // // 1 1 0 1 1 // 1 0 0 1 1 // 1 1 0 0 1 // 1 1 1 0 1 // 1 0 1 1 1
    stopMoving();
    delay(1000);
    Serial1.println("10");
    break;
  default:
    moveForward(); // case 11
    break;
  }

    if (getDistance() < FrontDistanceTreshold)
  {
    avoidObstacle();
  }

}

//==================================== FUNCTIONS =====================================================

void moveForward() // Move forward
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void moveBackward() // Move backward
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void turnLeft() // Turn Left
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, speedLeft);
  analogWrite(RIGHT_FOR, speedLeft);
  analogWrite(RIGHT_BACK, LOW);
}

void turnRight() // Turn Right
{
  analogWrite(LEFT_FOR, speedRight);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, speedRight);
}

void stopMoving() // Stop movement
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}

float getDistance()
{
  float distance;
  pinMode(UltrasonicPin, OUTPUT);
  digitalWrite(UltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicPin, LOW);
  pinMode(UltrasonicPin, INPUT);
  distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
  return distance;
}

void avoidObstacle()
{

  stopMoving(); //стоп
  delay(250);
  speedLeft = rightTimedTurnSpeedLeft;
  speedRight = rightTimedTurnSpeedRight;
  turnRight();
  delay(right90turnTiming); // Right 90 degree turn timing
  stopMoving();
  delay(100);
  speedLeft = LeftSpeed;
  speedRight = RightSpeed;
  moveForward();            // move 20 cm forward
  delay(forward20cmTiming); // 750
  stopMoving();
  delay(100);
  speedLeft = leftTimedTurnSpeedLeft;
  speedRight = leftTimedTurnSpeedRight;
  turnLeft();
  delay(left90turnTiming); // Turn 90 degrees left
  stopMoving();
  delay(100);
  speedLeft = LeftSpeed;
  speedRight = RightSpeed;
  moveForward();
  delay(forward30cmTiming); // Move 30 cm forward
  stopMoving();
  delay(100);
  speedLeft = leftTimedTurnSpeedLeft;
  speedRight = leftTimedTurnSpeedRight;
  turnLeft();              // turn left about 80 degrees
  delay(left80turnTiming); //
  stopMoving();
  delay(100);
  leftEdge = digitalRead(LN_SENS_PIN_RIGHTEDGE);
  left = digitalRead(LN_SENS_PIN_RIGHT);
  mid = digitalRead(LN_SENS_PIN_MIDDLE);
  right = digitalRead(LN_SENS_PIN_LEFT);
  rightEdge = digitalRead(LN_SENS_PIN_LEFTEDGE);
  while ((leftEdge == 0) && (left == 0) && (mid == 0) && (right == 0) && (rightEdge == 0))
  {
    speedLeft = LeftSpeed;
    speedRight = RightSpeed;
    moveForward();
    leftEdge = digitalRead(LN_SENS_PIN_RIGHTEDGE);
    left = digitalRead(LN_SENS_PIN_RIGHT);
    mid = digitalRead(LN_SENS_PIN_MIDDLE);
    right = digitalRead(LN_SENS_PIN_LEFT);
    rightEdge = digitalRead(LN_SENS_PIN_LEFTEDGE);
  }
  stopMoving();
  delay(100);
  speedLeft = rightTimedTurnSpeedLeft; // turn right to the line
  speedRight = rightTimedTurnSpeedRight;
  turnRight();
  delay(right25turnTiming); // 150
}

int getCurrentstate()
{
  if ((leftEdge == 0) && (left == 0) && (mid == 1) && (right == 0) && (rightEdge == 0)) //            0 0 1 0 0
  {
    currentState = 1;
    Serial1.println("case1");
  }
  else if ((leftEdge == 0) && (left == 1) && (mid == 1) && (right == 0) && (rightEdge == 0)) //       0 1 1 0 0
  {
    currentState = 2;
    Serial1.println("case2");
  }
  else if ((leftEdge == 0) && (left == 1) && (mid == 0) && (right == 0) && (rightEdge == 0)) //       0 1 0 0 0
  {
    currentState = 3;
    Serial1.println("case3");
  }
  else if ((leftEdge == 1) && (left == 1) && (mid == 0) && (right == 0) && (rightEdge == 0)) //       1 1 0 0 0
  {
    currentState = 4;
    Serial1.println("case4");
  }
  else if ((leftEdge == 1) && (left == 0) && (mid == 0) && (right == 0) && (rightEdge == 0)) //        1 0 0 0 0
  {
    currentState = 5;
    Serial1.println("case5");
  }
  else if ((leftEdge == 0) && (left == 0) && (mid == 1) && (right == 1) && (rightEdge == 0)) //        0 0 1 1 0
  {
    currentState = 6;
    Serial1.println("case6");
  }
  else if ((leftEdge == 0) && (left == 0) && (mid == 0) && (right == 1) && (rightEdge == 0)) //        0 0 0 1 0
  {
    currentState = 7;
    Serial1.println("case7");
  }
  else if ((leftEdge == 0) && (left == 0) && (mid == 0) && (right == 1) && (rightEdge == 1)) //        0 0 0 1 1
  {
    currentState = 8;
    Serial1.println("case8");
  }
  else if ((leftEdge == 0) && (left == 0) && (mid == 0) && (right == 0) && (rightEdge == 1)) //        0 0 0 0 1
  {
    currentState = 9;
    Serial1.println("case9");
  }
  else if (((leftEdge == 1) && (left == 1) && (mid == 0) && (right == 1) && (rightEdge == 1)) || // 1 1 0 1 1
           ((leftEdge == 1) && (left == 0) && (mid == 0) && (right == 1) && (rightEdge == 1)) || // 1 0 0 1 1
           ((leftEdge == 1) && (left == 1) && (mid == 0) && (right == 0) && (rightEdge == 1)) || // 1 1 0 0 1
           ((leftEdge == 1) && (left == 1) && (mid == 1) && (right == 0) && (rightEdge == 1)) || // 1 1 1 0 1
           ((leftEdge == 1) && (left == 0) && (mid == 1) && (right == 1) && (rightEdge == 1)))   // 1 0 1 1 1
  {
    currentState = 10;
  }
  else
  {
    currentState = 11;
  }

  return currentState;
}