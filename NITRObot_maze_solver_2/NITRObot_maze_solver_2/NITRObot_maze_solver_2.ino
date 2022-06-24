
/*
      NITRO Clubs EU - Network of IcT Robo Clubs
 
 WEB site: https://www.nitroclubs.eu 
 GitHub repositories: https://github.com/nitroclubs?tab=repositories 
 
          NITRObot Maze solver Version 2.0
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


// Robot parameters:
// NITRObot length is 25.0 cm.
// NITRObot width is  16.7 cm.

// Maze parameters:
// In order for the robot to be able to safely make an U turn,
// we will choose the maze width to be 3 times the robot width,
// which is equal to 50.1, we will approximate this value to 50 cm.

//====== INCLUDE ======
#include <Arduino.h>

#include <Servo.h> // Servo library

//====== DEFINE ======
#define MOTOR_LEFT_FWD_PIN 9    // PWMB
#define MOTOR_LEFT_BKWD_PIN 5   // DIRB  ---  Left
#define MOTOR_RIGHT_FWD_PIN 6   // PWMA
#define MOTOR_RIGHT_BKWD_PIN 10 // DIRA  ---  Right

//====== CONSTANTS ======
const int UltrasonicPin = 3;
const int ServoPin = 13;
const int LeftAvoidancePin = 12;
const int RightAvoidancePin = A5;
const int LeftLightPin = A3;
const int RightLightPin = A4;

// Servo parameters
const int FrontServoAngle = 90;
const int SideServoAngle = 0;         // to the right for Right hand wall folowing
const int servoTiming90degrees = 225; // It takes 150 ms for SG90 Micro servo to rotate 60 degrees, sot it will take 225 ms for 90 degree rotation.

//*****************************************************************

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
// const int slightTurnSpeedCorrection = 200;
// const int sharpTurnSpeedCorrection = 220;
// const int leadingWheelAgresiveTurnSpeed = 255;

//////////// END SETTINGS FOR NITRObot number one //////////////////
////////////////////////////////////////////////////////////////////

//*****************************************************************

/////////////////////////////////////////////////////////////////////
// SETINGS FOR NITRObot number two (values aquired by using the NITRObot Bluetooth callibration utility)
// Uncomment the settings for the NITRObot in use!
// Do not forget to comment the settings for the other NITRObot!
//////////////////////////////////////

const int LeftSpeed = 67 + 31; 
const int RightSpeed = 67;     

const int rightTimedTurnSpeedLeft = 180;  // Left side speed at which the 90 degree turn to be executed
const int rightTimedTurnSpeedRight = 150; // Right side speed at which the 90 degree turn to be executed
const int right90turnTiming = 532;        // Delay in ms for complete 90 degree turn to be executed at the given speed above

const int leftTimedTurnSpeedLeft = 150;   // Left side speed at which the 90 degree turn to be executed
const int leftTimedTurnSpeedRight = 180;  // Right side speed at which the 90 degree turn to be executed
const int left90turnTiming = 496;         // Delay in ms for complete 90 degree turn to be executed at the given speed above

const int forward20cmTiming = 750; 
const int forward30cmTiming = 900; 

const float FrontDistanceTreshold = 30.0;
const float sideCorridorMinDistance = 43.0; 

const int slightTurnSpeedCorrection = 120; 
const int sharpTurnSpeedCorrection = 170;  
int robotPosition; 
float frontDistance, sideDistance, prevSideDistance;

bool caseUndefined = false;
bool evenPass = false;
int currentServoAngle = 90;

//////////// END SETTINGS FOR NITRObot number two //////////////////
////////////////////////////////////////////////////////////////////



//====== VARIABLES ======
int speedLeft = LeftSpeed;
int speedRight = RightSpeed;

//====== Instantiate (create) an instance of the Servo object (class), named servo
Servo servo;

//====== FORWARD FUNCTION DECLARATIONS ======
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void turn90degreesLeft();
void turn90degreesRight();
void stopMoving();
float getDistance(int servoAngle); // Read the Ultasonic Sensor pointing at the given servo angle

//-----------------------------------------------

void setup()
{
  pinMode(ServoPin, OUTPUT);
  pinMode(MOTOR_LEFT_FWD_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_BKWD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_BKWD_PIN, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LeftAvoidancePin, INPUT);
  pinMode(RightAvoidancePin, INPUT);
  pinMode(LeftLightPin, INPUT);
  pinMode(RightLightPin, INPUT);

  servo.attach(ServoPin);
  servo.write(90); // Move the servo to center position

  Serial1.begin(9600);
  delay(1000); // Delay the start of NITRObot's movement

  moveForward(); // Initial start of the movement
  delay(300);
}

//---------------------------------------------------------

void loop()
{
  caseUndefined = false;

  // Reading secuence in order to avoid unnessesary servo movement
  if (!evenPass)
  {
    frontDistance = getDistance(FrontServoAngle);
    sideDistance = getDistance(SideServoAngle);
  }
  else
  {
    sideDistance = getDistance(SideServoAngle);
    frontDistance = getDistance(FrontServoAngle);
  }

  //// exclude ultrasound false readings removal
  if (sideDistance >= sideCorridorMinDistance && sideDistance > prevSideDistance)
  {
    stopMoving(); 
    float sideTempDistance;
    sideTempDistance = getDistance(SideServoAngle);
    delay(250);
    sideDistance = (sideTempDistance + getDistance(SideServoAngle)) / 2;
    prevSideDistance = sideDistance;
    frontDistance = getDistance(FrontServoAngle);
    frontDistance = getDistance(FrontServoAngle);
    if (evenPass)
    {
      sideTempDistance = getDistance(SideServoAngle); // Move the servo to the current reading sequence position
    }
    moveForward();
  }

  if (frontDistance <= FrontDistanceTreshold) //  Close to the wall in front of the robot
  {
    robotPosition = 1;
  }
  else if (sideDistance >= sideCorridorMinDistance) // The wall on the right is far away - turn to the wall
  {
    robotPosition = 2;
  }
  // Shallow right
  else if (sideDistance < 35.0 && sideDistance >= 29.0) //    |_________|__ROBOT__|_________|_________|_________|     The robot is to the left from the centerline treshold
  {                                                     //    50cm      35cm      29cm      21cm      15cm      0cm
    robotPosition = 3;
  }
  // Shallow left
  else if (sideDistance > 15.0 && sideDistance <= 21.0) //    |_________|_________|_________|__ROBOT__|_________|     The robot is to the right from the centerline treshold
  {                                                     //    50cm      35cm      29cm      21cm      15cm      0cm
    robotPosition = 4;
  }
  // Agresive left
  else if (sideDistance <= 15.0) //    |_________|_________|_________|_________|__ROBOT__|     The robot is on the far right of the corridor
  {                              //    50cm      35cm      29cm      21cm      15cm      0cm
    robotPosition = 5;
  }
  // Aggesive right
  else if (sideDistance >= 35.0) //    |__ROBOT__|_________|_________|_________|_________|     The robot is on the far left of the corridor
  {                              //    50cm      35cm      29cm      21cm      15cm      0cm
    robotPosition = 6;
  }
  else if (sideDistance >= 21.0 && sideDistance < 29.0) //    |_________|_________|__ROBOT__|_________|_________|     The robot is inside the center line threshold
  {                                                     //    50cm      35cm      29cm      21cm      15cm      0cm
    robotPosition = 7;
  }

  switch (robotPosition)
  {
  case 1: // Turn 90 degrees left // Close to the wall in front of the robot
    stopMoving();
    delay(50);
    // turn on the spot!!!
    turn90degreesLeft();
    break;
  case 2: // Turn 90 degrees right // The wall on the right is far away >= 50 cm. - turn to the wall
    turn90degreesRight();
    break;
  case 3: // Turn slight right
    speedLeft = speedLeft + slightTurnSpeedCorrection;
    if (speedLeft > 255)
    {
      speedLeft = 255;
    }

    turnRight();
    delay(70);
    break;
  case 4: // Turn slight left
    speedRight = speedRight + slightTurnSpeedCorrection;
    if (speedRight > 255)
    {
      speedRight = 255;
    }

    turnLeft();
    delay(70);
    break;
  case 5: // Turn  more agressive to the left
    speedRight = speedRight + sharpTurnSpeedCorrection;
    if (speedRight > 255)
    {
      speedRight = 255;
    }

    turnLeft();
    delay(100);
    break;
  case 6: // Turn  more agressive to the right
    speedLeft = speedLeft + sharpTurnSpeedCorrection;
    if (speedLeft > 255)
    {
      speedLeft = 255;
    }

    turnRight();
    delay(100);
    break;
  case 7: // Continue moving forward
          // Nothing to do here, continue with the execution of the loop
    break;
  default:
    break;
  }

  speedRight = RightSpeed;
  speedLeft = LeftSpeed;
  moveForward();
}

//============== FUNCTION DEFINITIONS ==============

void moveForward() // Move forward
{
  analogWrite(MOTOR_LEFT_FWD_PIN, abs(speedLeft));
  analogWrite(MOTOR_LEFT_BKWD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_FWD_PIN, abs(speedRight));
  analogWrite(MOTOR_RIGHT_BKWD_PIN, LOW);
}

void moveBackward() // Move backward
{
  analogWrite(MOTOR_LEFT_FWD_PIN, LOW);
  analogWrite(MOTOR_LEFT_BKWD_PIN, abs(speedLeft));
  analogWrite(MOTOR_RIGHT_FWD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_BKWD_PIN, abs(speedRight));
}

void turnLeft() // Turn Left
{
  analogWrite(MOTOR_LEFT_FWD_PIN, LOW);
  analogWrite(MOTOR_LEFT_BKWD_PIN, speedLeft);
  analogWrite(MOTOR_RIGHT_FWD_PIN, speedRight);
  analogWrite(MOTOR_RIGHT_BKWD_PIN, LOW);
}

void turnRight() // Turn Right
{
  analogWrite(MOTOR_LEFT_FWD_PIN, speedLeft);
  analogWrite(MOTOR_LEFT_BKWD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_FWD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_BKWD_PIN, speedRight);
}

void stopMoving() // Stop movement
{
  analogWrite(MOTOR_LEFT_FWD_PIN, HIGH);
  analogWrite(MOTOR_LEFT_BKWD_PIN, HIGH);
  analogWrite(MOTOR_RIGHT_FWD_PIN, HIGH);
  analogWrite(MOTOR_RIGHT_BKWD_PIN, HIGH);
}

float getDistance(int servoAngle) // Read the Ultasonic Sensor pointing at the given servo angle
{
  float distance = 0;
  if (servoAngle != currentServoAngle)
  {
    servo.write(servoAngle);
    delay(150); 
  }

  for (size_t i = 0; i < 7; i++)
  {
    pinMode(UltrasonicPin, OUTPUT);
    digitalWrite(UltrasonicPin, LOW);
    delayMicroseconds(2);
    digitalWrite(UltrasonicPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(UltrasonicPin, LOW);
    pinMode(UltrasonicPin, INPUT);
    distance = distance + pulseIn(UltrasonicPin, HIGH) / 58.00;
  }
  currentServoAngle = servoAngle;
  return distance / 7;
}

void turn90degreesRight()
{
  stopMoving(); //с
  delay(50);
  float frontTempDistance, sideTempDistance;
  sideTempDistance = getDistance(SideServoAngle);
  delay(250);
  sideTempDistance = (sideTempDistance + getDistance(SideServoAngle)) / 2;

  frontTempDistance = getDistance(FrontServoAngle);
  frontTempDistance = getDistance(FrontServoAngle);

  if (sideTempDistance >= sideCorridorMinDistance) 
  {
    speedRight = RightSpeed;
    speedLeft = LeftSpeed;
    moveForward();
    delay(750); // Move some more distance to the front
    stopMoving();
    speedLeft = rightTimedTurnSpeedLeft;
    speedRight = rightTimedTurnSpeedRight;
    turnRight();
    delay(right90turnTiming); // Right 90 degree turn timing
    speedRight = RightSpeed;  // Enter the corridor
    speedLeft = LeftSpeed;
    if (evenPass)
    {
      sideTempDistance = getDistance(SideServoAngle); // Move the servo to the current reading sequence position
    }
    moveForward();
    delay(750); 
    stopMoving();
    delay(50);
  }
  else
  {
    // RIGHT turn SKIPPED!
  }
}

void turn90degreesLeft()
{

  Serial1.println("Left 90 turn");

  stopMoving(); //
  delay(50);
  float frontTempDistance;
  frontTempDistance = getDistance(FrontServoAngle);
  if (frontTempDistance <= 15) 
  {
    speedRight = RightSpeed;
    speedLeft = LeftSpeed;
    moveBackward();
    delay(200); // Back up a bit!
    stopMoving();
  }
  speedLeft = leftTimedTurnSpeedLeft;
  speedRight = leftTimedTurnSpeedRight;
  turnLeft();
  delay(left90turnTiming); 
  stopMoving();
  delay(50);
}
