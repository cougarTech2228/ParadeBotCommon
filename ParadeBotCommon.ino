#include <Servo.h>
#include <FastLED.h>
#include <color.h>
#include <PS2X_lib.h>

// The library for driving the LED strips, FastLED can
// be found here: https://github.com/FastLED/FastLED

// We are using the version of the PS2X library from here:
// https://github.com/madsci1016/Arduino-PS2X
//
// This version returns a boolean from the read_gamepad method
// which allows us to ensure that the robot and controller are
// still communicating and, if not, stop the drive motors.

// For PS2 button presses to be recognized, 100 ms seems pretty good

static const int ADDITIONAL_LOOP_DELAY_MS = 100; // For PS2 button presses to be recognized, 100 ms seems pretty good
static const int UPDATES_PER_SECOND = 10;
static const int LOOP_DELAY_FOR_PS2_BUTTON_DEBOUNCE_MS = 100;
static const int LOOP_COUNTS_TO_HOLD_CANDY_SOLENOID_PIN_ACTIVE = 10;

static const int SERVO_FULL_REVERSE = 160;
static const int SERVO_STOPPED = 90;
static const int SERVO_FULL_FORWARD = 20;
static const int SERVO_DEADBAND = 2;
static const int SERVO_SAFETY_MARGIN = 20;

static const double PS2_CONTROLLER_MINIMUM_VALUE = 0.0;   // Stick is all the way up
static const double PS2_CONTROLLER_NEUTRAL_VALUE = 127.0; // Stick is centered
static const double PS2_CONTROLLER_MAXIMUM_VALUE = 255.0; // Stick is all the way down

static const int CANDY_SHOOTER_SOLENOID_PIN = 12;
static const int LEFT_DRIVE_MOTOR_PIN = 10;
static const int RIGHT_DRIVE_MOTOR_PIN = 9;
static const int CANDY_SHOOTER_MOTOR_PIN = 8;

static const int CANDY_SHOOTER_MOTOR_STATE_OFF = 0;
static const int CANDY_SHOOTER_MOTOR_STATE_ON = 1;
static const int CANDY_SHOOTER_MOTOR_SPEED = 150;

static const int PARADE_BOT_TYPE_PIN = 4;

static const int CHILD_MODE_LED_PIN = 6;
static const int CHILD_SWITCH_PIN = 7;
static const int CHILD_MODE_SAFETY_MARGIN = 20;

static const int PS2_CONTROLLER_CLOCK_PIN = 13;
static const int PS2_CONTROLLER_COMMAND_PIN = 32;
static const int PS2_CONTROLLER_ATTENTION_PIN = 30;
static const int PS2_CONTROLLER_DATA_PIN = 38;

static const int MIN_PWM_SIGNAL_WIDTH = 1000;
static const int MAX_PWM_SIGNAL_WIDTH = 2000;

static const int TURN_COMPENSATION = 7;

// LED Constants
static const int LED_PIN = 5;
static const int NUM_LEDS = 120;
static const int BRIGHTNESS = 64;
static const int HEAT_INDEX = 255;

static const int TURN_CORRECTION = 10;

#define LED_TYPE WS2811
#define COLOR_ORDER GRB


// PS2 Game Controller Instance
PS2X ps2x;
byte vibrate = 0;
CRGB leds[NUM_LEDS];

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

enum LEDPresets
{
  fullRed, fullWhite, fullYellow, fullGreen, fullBlue,
  fillRedAndGreen, flashRedAndGreen, redWhiteAndBlue, rainbow,
  flashRainbow
};

static uint8_t startIndex = 0;
DEFINE_GRADIENT_PALETTE( heatmap_gp )
{
  //0,     0,    0,    0,   //black
  //128,   20
  5,  0,    0,   //red
  //224,   255,  255,  0,   //bright yellow
  255,   255,  255,  255
}; //full white
CRGBPalette16 myPal = heatmap_gp;

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
  delay(3000); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  ps2x.config_gamepad(PS2_CONTROLLER_CLOCK_PIN, PS2_CONTROLLER_COMMAND_PIN,
                      PS2_CONTROLLER_ATTENTION_PIN, PS2_CONTROLLER_DATA_PIN,
                      true, true);

  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

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

      doLeds();
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

  FastLED.show();
  FastLED.delay(UPDATES_PER_SECOND);
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
  static const double PIV_Y_LIMIT = 32.0;

  // TEMP VARIABLES
  double   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  double   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  int      nPivSpeed;      // Pivot Speed                          (-128..+127)
  double   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

  /*
    Serial.print("LY: ");
    Serial.print(LY);
    Serial.print(" RX: ");
    Serial.println(RX);
  */

  nJoyY = map(LY, PS2_CONTROLLER_MINIMUM_VALUE, PS2_CONTROLLER_MAXIMUM_VALUE, 127, -128);
  nJoyX = map(RX, PS2_CONTROLLER_MINIMUM_VALUE, PS2_CONTROLLER_MAXIMUM_VALUE, -128, 127);

  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0)
  {
    // Forward
    if (nJoyX >= 0)
    {
      nMotPremixL = 127.0;
      nMotPremixR = 127.0 - nJoyX;
    }
    else
    {
      nMotPremixL = 127.0 + nJoyX;
      nMotPremixR = 127.0;
    }
  }
  else
  {
    // Reverse
    if (nJoyX >= 0)
    {
      nMotPremixL = 127 - nJoyX;
      nMotPremixR = 127.0;
    }
    else
    {
      nMotPremixL = 127.0;
      nMotPremixR = 127.0 + nJoyX;
    }
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY / 128.0;
  nMotPremixR = nMotPremixR * nJoyY / 128.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;

  if (abs(nJoyY) > PIV_Y_LIMIT)
  {
    fPivScale = 0.0;
  }
  else
  {
    fPivScale = (1.0 - abs(nJoyY) / PIV_Y_LIMIT);
  }

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

  /*
    Serial.print("nJoyY: " );
    Serial.print(nJoyY);
    Serial.print(" nPivSpeed: " );
    Serial.print(nPivSpeed);
  */

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

  //Serial.print(isNewParadeBot);

  if (isNewParadeBot == false)
  {
    // On the old parade bot, for some reason the right motor
    // is faster than the left so we're going to give the left
    // motor a little extra gas here ...
    if (nPivSpeed == 0) // We are NOT turning
    {
      if (leftMotorAngle > SERVO_STOPPED)
      {
        leftMotorAngle += TURN_COMPENSATION;
      }
      else if (leftMotorAngle < SERVO_STOPPED)
      {
        leftMotorAngle -= TURN_COMPENSATION;
      }
      else
      {
        // do nothing
      }
    }
  }
  else
  {
    if (nPivSpeed == 0) // We are NOT turning
    {
      if (rightMotorAngle > SERVO_STOPPED)
      {
        rightMotorAngle += TURN_COMPENSATION;
      }
      else if (rightMotorAngle < SERVO_STOPPED)
      {
        rightMotorAngle -= TURN_COMPENSATION;
      }
      else
      {
        // do nothing
      }
    }
  }

  // When approaching maximum forward or reverse speed and
  // you also apply maximum turn in one direction or the
  // other, one of the two motors is commanded to its 'stop'
  // position. This has the effect of not being able to turn
  // at high speeds. We want to clip the settings such that
  // each motor is never commanded to completely stop when
  // the robot is turning. We will add or subtract TURN_CORRECTION
  // just to give the motor a little bit of juice and hopefully 
  // keep it out of full stop/brake mode.

  // First make sure we're not just turning in place
  if ((nJoyY != 0) && (nPivSpeed != 0))
  {
    //Serial.println("Entering first if");
    // Correct Left Motor
    if (nJoyY > 0) // moving forward so leftMotorAngle is 90 to 180 here
    {
      //Serial.println("Entering second if");
      if ((leftMotorAngle >= SERVO_STOPPED) &&
          (leftMotorAngle <= (SERVO_STOPPED + SERVO_DEADBAND)))
      {
        leftMotorAngle = SERVO_STOPPED + SERVO_DEADBAND + TURN_CORRECTION;
      }
    }
    else // nJoy < 0, moving backward so leftMotorAngle is 0 to 90 here
    {
      if ((leftMotorAngle <= SERVO_STOPPED) &&
          (leftMotorAngle >= (SERVO_STOPPED - SERVO_DEADBAND)))
      {
        leftMotorAngle = SERVO_STOPPED - SERVO_DEADBAND - TURN_CORRECTION;
      }
    }

    // Correct Right Motor
    if (nJoyY > 0) // moving forward so rightMotorAngle is 0 to 90 here
    {
      if ((rightMotorAngle <= SERVO_STOPPED) &&
          (rightMotorAngle >= (SERVO_STOPPED - SERVO_DEADBAND)))
      {
        rightMotorAngle = SERVO_STOPPED - SERVO_DEADBAND - TURN_CORRECTION;
      }
    }
    else // nJoy < 0, moving backward so rightMotorAngle is 90 to 180 here
    {
      if ((rightMotorAngle >= SERVO_STOPPED) &&
          (rightMotorAngle <= (SERVO_STOPPED + SERVO_DEADBAND)))
      {
        rightMotorAngle = SERVO_STOPPED + SERVO_DEADBAND + TURN_CORRECTION;
      }
    }
  }

  /*
    Serial.print(" **** Writing leftMotorAngle: " );
    Serial.print(leftMotorAngle);
    Serial.print(" **** Writing rightMotorAngle: " );
    Serial.println(rightMotorAngle);
  */

  leftDriveMotor.write(leftMotorAngle);
  rightDriveMotor.write(rightMotorAngle);
}

/**************************************************************
   doLeds()
 **************************************************************/
void doLeds()
{
  if (ps2x.ButtonPressed(PSB_CIRCLE))
  {
    Serial.println("Circle is pressed");
    fillLEDsRed();
  }
  else if (ps2x.ButtonPressed(PSB_L2))
  {
    Serial.println("L2 is pressed");
    fillLEDsWhite();
  }
  else if (ps2x.ButtonPressed(PSB_TRIANGLE))
  {
    Serial.println("Triangle is pressed");
    fillLEDsGreen();
  }
  else if (ps2x.ButtonPressed(PSB_CROSS))
  {
    Serial.println("Cross is pressed");
    fillLEDsYellow();
  }
  else if (ps2x.ButtonPressed(PSB_SQUARE))
  {
    Serial.println("Square is pressed");
    rainbowLEDs();
  }
  else if (ps2x.ButtonPressed(PSB_PAD_UP))
  {
    Serial.println("Up is pressed");
    fillLEDsRedWhiteAndBlue();
  }
  else if (ps2x.ButtonPressed(PSB_PAD_DOWN))
  {
    Serial.println("Down is pressed");
    fillLEDsBlue();
  }
  else if (ps2x.ButtonPressed(PSB_PAD_RIGHT))
  {
    Serial.println("Right is pressed");
    flashingRedAndGreen();
  }
  else if (ps2x.ButtonPressed(PSB_PAD_LEFT))
  {
    Serial.println("Left is pressed");
    redAndGreen();
  }
  else if (ps2x.ButtonPressed(PSB_R2))
  {
    Serial.println("R2 is pressed");
    flashingRainbow();
  }
}

/**************************************************************
   rainbowLEDs()
 **************************************************************/
void rainbowLEDs()
{
  startIndex = startIndex + 1; /* motion speed */
  currentPalette = RainbowStripeColors_p;
  currentBlending = LINEARBLEND;
  uint8_t brightness = 255;

  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, startIndex, brightness, currentBlending);
    startIndex += 5;
  }
}

/**************************************************************
   fillLEDsRed()
 **************************************************************/
void fillLEDsRed()
{
  currentBlending = LINEARBLEND;
  currentPalette = RainbowStripeColors_p;
  uint8_t brightness = 255;
  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, 0, brightness, currentBlending);
  }
}

/**************************************************************
   fillLEDsWhite()
 **************************************************************/
void fillLEDsWhite()
{
  currentBlending = LINEARBLEND;
  currentPalette = RainbowStripeColors_p;
  uint8_t brightness = 255;
  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette(myPal, HEAT_INDEX);
  }
}

/**************************************************************
   fillLEDsGreen()
 **************************************************************/
void fillLEDsGreen()
{
  currentBlending = LINEARBLEND;
  currentPalette = RainbowStripeColors_p;
  uint8_t brightness = 255;
  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, 95, brightness, currentBlending);
  }
}

/**************************************************************
   fillLEDsBlue()
 **************************************************************/
void fillLEDsBlue()
{
  currentBlending = LINEARBLEND;
  currentPalette = RainbowStripeColors_p;
  uint8_t brightness = 255;
  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, 120, brightness, currentBlending);
  }
}

/**************************************************************
   fillLEDsYellow()
 **************************************************************/
void fillLEDsYellow()
{
  currentBlending = LINEARBLEND;
  currentPalette = RainbowStripeColors_p;
  uint8_t brightness = 255;
  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, 60, brightness, currentBlending);
  }
}

/**************************************************************
   fillLEDsRedWhiteAndBlue()
 **************************************************************/
void fillLEDsRedWhiteAndBlue()
{
  startIndex = startIndex + 20; /* motion speed */
  uint8_t brightness = 255;
  currentPalette = myRedWhiteBluePalette_p;
  currentBlending = LINEARBLEND;
  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, startIndex, brightness, currentBlending);
    startIndex += 1;
  }
}

/**************************************************************
   redAndGreen()
 **************************************************************/
void redAndGreen()
{
  startIndex = startIndex + 20; /* motion speed */

  currentBlending = LINEARBLEND;
  setupRedAndGreenPalette();
  uint8_t brightness = 255;

  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, startIndex, brightness, currentBlending);
    startIndex += 3;
  }
}

/**************************************************************
   flashingRedAndGreen()
 **************************************************************/
void flashingRedAndGreen()
{
  startIndex = startIndex + 10; /* motion speed */

  currentBlending = NOBLEND;
  setupRedAndGreenPalette();
  uint8_t brightness = 255;

  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, startIndex, brightness, currentBlending);
  }
}

/**************************************************************
   flashingRainbow()
 **************************************************************/
void flashingRainbow()
{
  startIndex = startIndex + 5; /* motion speed */
  currentPalette = RainbowStripeColors_p;
  currentBlending = LINEARBLEND;
  uint8_t brightness = 255;

  for ( int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette( currentPalette, startIndex, brightness, currentBlending);
  }
}

// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
  CRGB::Red,
  CRGB::Gray, // 'white' is too bright compared to red and blue
  CRGB::Blue,
  CRGB::Black,

  CRGB::Red,
  CRGB::Gray,
  CRGB::Blue,
  CRGB::Black,

  CRGB::Red,
  CRGB::Red,
  CRGB::Gray,
  CRGB::Gray,
  CRGB::Blue,
  CRGB::Blue,
  CRGB::Black,
  CRGB::Black
};

/**************************************************************
   setupRedAndGreenPalette()
 **************************************************************/
void setupRedAndGreenPalette()
{
  CRGB green  = CHSV( HUE_GREEN, 255, 255);
  CRGB red = CHSV( HUE_RED, 255, 255);
  CRGB white  = CRGB::White;
  currentPalette = CRGBPalette16(
                     green, green, green, white,
                     white, red, red, red,
                     green, green, green, white,
                     white, red, red, red);
}
