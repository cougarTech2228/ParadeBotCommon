#include <PS2X_lib.h>
#include <Servo.h>

// The library for driving the LED strips, FastLED can
// be found here: https://github.com/FastLED/FastLED

// We are using the version of the PS2X library from here:
// https://github.com/madsci1016/Arduino-PS2X
//
// This version returns a boolean from the read_gamepad method
// which allows us to ensure that the robot and controller are
// still communicating and, if not, stop the drive motors.

// For PS2 button presses to be recognized, 100 ms seems pretty good
#define LOOP_DELAY_FOR_PS2_BUTTON_DEBOUNCE_MS 100

#define LOOP_COUNTS_TO_HOLD_CANDY_SOLENOID_PIN_ACTIVE 10

#define SERVO_FULL_REVERSE 180
#define SERVO_STOPPED 90
#define SERVO_FULL_FORWARD 0
#define SERVO_DEADBAND 2
#define SERVO_SAFETY_MARGIN 20

#define PS2_CONTROLLER_MINIMUM_VALUE 0.0   // Stick is all the way up
#define PS2_CONTROLLER_NEUTRAL_VALUE 127.0 // Stick is centered 
#define PS2_CONTROLLER_MAXIMUM_VALUE 255.0 // Stick is all the way down

#define CANDY_SHOOTER_SOLENOID_PIN 12
#define LEFT_DRIVE_MOTOR_PIN 10
#define RIGHT_DRIVE_MOTOR_PIN 9
#define CANDY_SHOOTER_MOTOR_PIN 8

#define CANDY_SHOOTER_MOTOR_STATE_OFF 0
#define CANDY_SHOOTER_MOTOR_STATE_ON 1
#define CANDY_SHOOTER_MOTOR_SPEED 150

#define PARADE_BOT_TYPE_PIN 4

#define CHILD_MODE_LED_PIN 6
#define CHILD_SWITCH_PIN 7
#define CHILD_MODE_SAFETY_MARGIN 40

#define PS2_CONTROLLER_CLOCK_PIN 13
#define PS2_CONTROLLER_COMMAND_PIN 32
#define PS2_CONTROLLER_ATTENTION_PIN 30
#define PS2_CONTROLLER_DATA_PIN 38

#define MIN_PWM_SIGNAL_WIDTH 1000
#define MAX_PWM_SIGNAL_WIDTH 2000

// PS2 Game Controller Instance
PS2X ps2x;

byte vibrate = 0;

double RY = PS2_CONTROLLER_NEUTRAL_VALUE;
double RX = PS2_CONTROLLER_NEUTRAL_VALUE;
double LY = PS2_CONTROLLER_NEUTRAL_VALUE;

int candyShooterMotorState = CANDY_SHOOTER_MOTOR_STATE_OFF;

bool isChildModeOn = true;
bool isNewParadeBot = false;

// These variables will hold the adjusted forward and reverse speeds
// based on the current Child Mode setting. We've also found that full
// forward and reverse was too fast to control the robot safely so
// we are going to decrease the max range here. The Child Mode setting
// will always less than this initial value.
int adjustedServoFullForward = SERVO_FULL_FORWARD + SERVO_SAFETY_MARGIN;
int adjustedServoFullReverse = SERVO_FULL_REVERSE - SERVO_SAFETY_MARGIN;

int solenoidLoopCounter = 0;

Servo rightDriveMotor, leftDriveMotor, candyShooterMotor;

/**************************************************************
   setup()
 **************************************************************/
void setup()
{
  Serial.begin(9600);

  // Set the pin modes
  pinMode(PARADE_BOT_TYPE_PIN, INPUT_PULLUP);
  pinMode(CHILD_SWITCH_PIN, INPUT_PULLUP);
  pinMode(CHILD_MODE_LED_PIN, OUTPUT);

  // We are using a pin on the Arduino tied to ground to determine
  // whether or not we're running on the new parade bot.
  if (digitalRead(PARADE_BOT_TYPE_PIN) == LOW)
  {
    isNewParadeBot = true;

    // Attach the shooter motor to its respective pins, with min/max PWM signal widths
    candyShooterMotor.attach(CANDY_SHOOTER_MOTOR_PIN,
                             MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);

    // Set the shooter motor to its "stopped" position
    candyShooterMotor.write(SERVO_STOPPED);
  }
  else // Old Parade Bot
  {
    isNewParadeBot = false;

    pinMode(CANDY_SHOOTER_SOLENOID_PIN, OUTPUT);

    // Turn candy shooter solenoid off to start
    digitalWrite(CANDY_SHOOTER_SOLENOID_PIN, LOW);
  }

  // Attach the drive motors to their respective pins, with min/max PWM signal widths
  leftDriveMotor.attach(LEFT_DRIVE_MOTOR_PIN,
                        MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);
  rightDriveMotor.attach(RIGHT_DRIVE_MOTOR_PIN,
                         MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);

  // Set the drive motors to their "stopped" position
  leftDriveMotor.write(SERVO_STOPPED);
  rightDriveMotor.write(SERVO_STOPPED);

  // Setup controller pins and settings:
  // GamePad(clock, command, attention, data, Pressures?, Rumble?)
  ps2x.config_gamepad(PS2_CONTROLLER_CLOCK_PIN, PS2_CONTROLLER_COMMAND_PIN,
                      PS2_CONTROLLER_ATTENTION_PIN, PS2_CONTROLLER_DATA_PIN,
                      true, true);
}

/**************************************************************
   loop()
 **************************************************************/
void loop()
{
  // Monitor Child Mode Switch
  handleChildModeSwitch();
 
  if (isChildModeOn)
  {
    Serial.print("CHILD - ");
    
    adjustedServoFullForward = SERVO_FULL_FORWARD + SERVO_SAFETY_MARGIN +
                               CHILD_MODE_SAFETY_MARGIN;
    adjustedServoFullReverse = SERVO_FULL_REVERSE - SERVO_SAFETY_MARGIN -
                               CHILD_MODE_SAFETY_MARGIN;
  }
  else
  {
      adjustedServoFullForward = SERVO_FULL_FORWARD + SERVO_SAFETY_MARGIN;
      adjustedServoFullReverse = SERVO_FULL_REVERSE - SERVO_SAFETY_MARGIN;
  }
  /*
  Serial.print(" adjServoFullFwd: ");
  Serial.print(adjustedServoFullForward);

  Serial.print(" adjServoFullRev: ");
  Serial.println(adjustedServoFullReverse);
  */
  // Read PS2 Controller
  if (ps2x.read_gamepad(false, vibrate))
  {
    LY = ps2x.Analog(PSS_LY); // Reading Left stick Y axis
    RX = ps2x.Analog(PSS_RX); // Reading Right stick X axis

    handleDriveMotors();
    
    if (ps2x.NewButtonState())
    {
      if (ps2x.ButtonPressed(PSB_R2))
      {
        //Serial.println("PSB_R2 button pressed");

        if (isNewParadeBot == true)
        {
          handleCandyShooterMotor();
        }
        else
        {
          // Set a counter to hold the line active and decrement
          // it during each loop iteration until it gets to zero.
          solenoidLoopCounter = LOOP_COUNTS_TO_HOLD_CANDY_SOLENOID_PIN_ACTIVE;
        }
      }
    }
  }
  else // PS2 Read Failed
  {
    Serial.println("PS2 Read Failed");
    leftDriveMotor.write(SERVO_STOPPED);
    rightDriveMotor.write(SERVO_STOPPED);
  }

  // We only need to worry about the candy shooter solenoid
  // on the old parade bot.
  if (isNewParadeBot == false)
  {
    handleCandyShooterSolenoid();
  }

  // It's possible that this delay could be moved inside the
  // if (ps2x.NewButtonState()) scope, we should try this
  delay(LOOP_DELAY_FOR_PS2_BUTTON_DEBOUNCE_MS);
}

/*************************************************************
   handleChildModeSwitch()
 **************************************************************/
void handleChildModeSwitch()
{
  if (digitalRead(CHILD_SWITCH_PIN) == HIGH)
  {
    isChildModeOn = true;
    digitalWrite(CHILD_MODE_LED_PIN, HIGH);
    //Serial.println("Child Mode On");
  }
  else
  {
    isChildModeOn = false;
    digitalWrite(CHILD_MODE_LED_PIN, LOW);
    //Serial.println("Child Mode Off");
  }
}

/**************************************************************
   handleCandyShooterSolenoid()
 **************************************************************/
void handleCandyShooterSolenoid()
{
  if (solenoidLoopCounter > 0)
  {
    //Serial.println("Solenoid Pin High");
    digitalWrite(CANDY_SHOOTER_SOLENOID_PIN, HIGH);
  }
  else
  {
    //Serial.println("Solenoid Pin Low");
    digitalWrite(CANDY_SHOOTER_SOLENOID_PIN, LOW);
  }

  //Serial.print("solenoidLoopCounter: ");
  //Serial.println(solenoidLoopCounter);

  solenoidLoopCounter--;
}

/**************************************************************
   handleCandyShooterMotor()
 **************************************************************/
void handleCandyShooterMotor()
{
  if (candyShooterMotorState == CANDY_SHOOTER_MOTOR_STATE_OFF)
  {
    candyShooterMotorState = CANDY_SHOOTER_MOTOR_STATE_ON;
    // Serial.print("Candy Shooter Motor On -->");
    // Serial.println();
  }
  else
  {
    candyShooterMotorState = CANDY_SHOOTER_MOTOR_STATE_OFF;
    //  Serial.print("<-- Candy Shooter Motor Off");
    //  Serial.println();
  }

  if (candyShooterMotorState == CANDY_SHOOTER_MOTOR_STATE_OFF)
  {
    candyShooterMotor.write(SERVO_STOPPED);
  }
  else
  {
    candyShooterMotor.write(CANDY_SHOOTER_MOTOR_SPEED);
  }
}

/**************************************************************
   handleDriveMotors()
 **************************************************************/
void handleDriveMotors()
{
  // This code was lifted from the following website:
  // https://www.impulseadventure.com/elec/robot-differential-steering.html
  // It has been left largely untouched, magic numbers and all.


  // INPUTS
  int     nJoyX;       // Joystick X input               (-128..+127)
  int     nJoyY;       // Joystick Y input               (-128..+127)

  // OUTPUTS
  int     nMotMixL;    // Motor (left)  mixed output     (-128..+127)
  int     nMotMixR;    // Motor (right) mixed output     (-128..+127)

  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  double fPivYLimit = 32.0;

  // TEMP VARIABLES
  double   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  double   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  int      nPivSpeed;      // Pivot Speed                          (-128..+127)
  double   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

  nJoyY = map(LY, PS2_CONTROLLER_MINIMUM_VALUE, PS2_CONTROLLER_MAXIMUM_VALUE, 127, -128);
  nJoyX = map(RX, PS2_CONTROLLER_MINIMUM_VALUE, PS2_CONTROLLER_MAXIMUM_VALUE, -128, 127);

  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0)
  {
    // Forward
    nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
  }
  else
  {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY / 128.0;
  nMotPremixR = nMotPremixR * nJoyY / 128.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

  // Convert to Motor PWM range
  int leftMotorAngle = SERVO_STOPPED;
  int rightMotorAngle = SERVO_STOPPED;

  //leftMotorAngle =
  //  map(nMotMixL, 127, -128, adjustedServoFullForward, adjustedServoFullReverse);

  // The left motor is mounted 180 degrees different from
  // the right motor, so we need to reverse the values being
  // sent to it. This eliminates the need for "null modem"
  // cables being installed inline to the left motors.
  leftMotorAngle =
    map(nMotMixL, 127, -128, adjustedServoFullReverse, adjustedServoFullForward);

  rightMotorAngle =
    map(nMotMixR, 127, -128, adjustedServoFullForward, adjustedServoFullReverse);
  
  // Seeing a 92 on right stick when idle so put a deadband on both
  // servo values
  if ((leftMotorAngle <= (SERVO_STOPPED + SERVO_DEADBAND)) &&
      (leftMotorAngle >= (SERVO_STOPPED - SERVO_DEADBAND)))
  {
    leftMotorAngle = SERVO_STOPPED;
  }

  if ((rightMotorAngle <= (SERVO_STOPPED + SERVO_DEADBAND)) &&
      (rightMotorAngle >= (SERVO_STOPPED - SERVO_DEADBAND)))
  {
    rightMotorAngle = SERVO_STOPPED;
  }

  leftDriveMotor.write(leftMotorAngle);
  rightDriveMotor.write(rightMotorAngle);

  /*
    Serial.print("Left Angle: ");
    Serial.print(leftMotorAngle);
    Serial.print("   Right Angle: ");
    Serial.println(rightMotorAngle);
  */
}
