#include <Arduino.h>
/*
      NITRO Clubs EU - Network of IcT Robo Clubs

 WEB site: https://www.nitroclubs.eu
 GitHub repositories: https://github.com/nitroclubs?tab=repositories

 
      NITRObot precise calibration over Bluetooth v1.0 
      ** Requires the use of an Android smart phone!  **

1. Connect the Bluetooth module to your NITRObot
2. Pair your Android smartphone to the Bluetooth module
3. Upload the file with .apk extention (the Android application) 
   from the MITappInventorFiles folder in this repository to your
   Android smart phone 
4. Install the aplication on your phone from the .apk file
5. Upload this sketch (.ino) to your NITRObot
6. Start the application on your smart phone
7. Switch on your NITRObot
8. Touch "Connect" on the application screen (on the smartphone)
9. Choose NITRObot's' Bluetooth name (ID) from the list
10. Use the text boxes on the screen to change the values for the speed of the left and right wheel and for the delay time (execution time)
11. Click the "Up arrow" and the robot will start moving forward (or turn) according to the wheel speed values set in the text boxes
    The movement will last for the number of milliseconds set in the "Delay time in ms.:" text box
12. IMPORTANT! Click the "Stop" button to stop the movement 
13. Every time, after the robot stops moving, press the "Stop" button - this is a requirement of the 
    communication protocol between the robot and the smart phone
14. Tune NITRObot to move stright forward for a chosen distance by changing the left and right 
    speed values and teh execution (delay) time accordingly
15. Tune NITRObot to make 90 degrees turn on the spot (to the left and to the right) at the deseired speed values

  !!!Alternativelly you can click the "Microphone" buton to use voice recognition to send the commands
  Say the followig phrases or words:
      "Go froward", "Go Back", "Stop"

  Note down all callibration values for you NITROobot - you will need this values when calibrating your 
  robot for tasks like "Line following" and "Maze solving"

****** Do not forget to press the "Stop" button after each robot movement - this is required by the protocol! ******

*/

///////////////////       IMPORTANT!    ///////////////////////////////////////////// 
/// In order to prolong the Life of the 18650 Battery:
/// DO NOT DISCHAREGE NITRObot's BATTERIES LOWER THAN 3.2V, WHICH IS 6.4V MESURED BY THE NITRObot's VOLTMETER (2x3.2V)
/// NEVER!!! NEVER DISCHARGE A BATTERY BELOW 2.8V, WHICH IS 5.6V (2x2.8v) MEASURED BY THE NITRObot's VOLTMETER  
///////////////////       IMPORTANT!    ///////////////////////////////////////////// 



//====== INCLUDE ======
#include <Arduino.h>

//====== DEFINITIONS ======
#define LEFT_FOR 9    // 
#define LEFT_BACK 5   // 
#define RIGHT_FOR 6   // 
#define RIGHT_BACK 10 // 

//====== CONSTANTS AND VARIABLES ======
const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3;
const int RgbPin = 2;
const int ServoPin = 13;
const int LedPin = 33;
const int LeftSpeed = 100;  // Initial speed value as in the Bluetooth app initial text box value
const int RightSpeed = 100; // Initial speed value as in the Bluetooth app initial text box value
const long MovementTime = 200;  // Initial turn time value as in the Bluetooth app initial text box value

int speedLeft = LeftSpeed;
int speedRight = RightSpeed;
unsigned long movementTime = MovementTime;
unsigned long currentMillis = 0;  //
unsigned long previousMillis = 0; //
bool isMoving = false;
int previousValue = 0;
char character = 'N';
char previousCharacter = 'N';
int value = 0;

//====== FUNCTION FORWARD DECLARATIONS ======
void moveForward();
void moveBackward();
void stopMoving();


//====== SETUP ======
void setup()
{
  pinMode(ServoPin, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LedPin, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(9600);
  delay(1000);
}


//====== LOOP ======
void loop()
{
  if (Serial1.available() > 0)
  {
    character = Serial1.read();
    value = Serial1.parseInt();
    Serial.flush();
  }

  currentMillis = millis();

  if (previousCharacter != character || previousValue != value)
  {
    previousCharacter = character;
    previousValue = value;

    if (character == 'R') // Set NITRObot's Right side speed value from the Bluetooth app text box
    {
      speedRight = value;
    }
    else if (character == 'L') // Set NITRObot's left side speed value  from the Bluetooth app text box
    {
      speedLeft = value;
    }
    else if (character == 'T') // Set NITRObot's execution time (delay time) value  from the Bluetooth app slider
    {
      movementTime = value;
    }
    else
    {

      if (value > 0)
      {
        switch (value)
        {
        case 1:
          if (!isMoving)
          {
            isMoving = true;
            previousMillis = currentMillis;
            moveForward();
          }
          break;
        case 4:
          if (!isMoving)
          {
            isMoving = true;
            previousMillis = currentMillis;
            moveBackward();
          }
          break;
        case 5:
          isMoving = false;
          stopMoving();
          break;

        default:
          isMoving = false;
          stopMoving();
          break;
        };
      }
    }
  }

  if (isMoving && (currentMillis - previousMillis >= movementTime))
  {
    stopMoving();
    isMoving = false;
  }
}


//====== FUNCTIONS ======
void moveForward()
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void moveBackward()
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void stopMoving()
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}
