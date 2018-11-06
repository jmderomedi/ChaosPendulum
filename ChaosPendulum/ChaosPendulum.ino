//#define ENCODER_OPTIMIZE_INTERRUPTS

//#include <SavLayFilter.h>
//#include <StepControl.h>
#include <elapsedMillis.h>
#include <font5x7.h>
#include <font8x16.h>
#include <fontlargenumber.h>
#include <sevenSegment.h>
#include <TeensyView.h>
#include <Encoder.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <math.h>

#define PIN_RESET 15
#define PIN_DC    5
#define PIN_CS    10
#define PIN_SCK   13
#define PIN_MOSI  11

AccelStepper motor(1, 17, 16);
//Stepper motor(17, 16);
Encoder buttonEnc(22, 23);
Encoder flyWheelEnc(7, 6);
TeensyView oled(PIN_RESET, PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI);


//SavLayFilter sgFilter();
//StepControl<> controller;

elapsedMillis timeElapsed;
elapsedMicros flyWheelTimer;

const int PPR = 4096;
const int ENCODERBUTTON = 18;

float fWOutput = 0.0;
float degPerPulse = 0.0;
float motorSpeed = 0.0;
float newPosition = 0.0;

float deltaPhi = 0.0;
float deltaTime = 0.0;

bool speedChange = false;

unsigned long buttonCounter = 0;

void setup() {
  Serial.begin(19200);
  pinMode(ENCODERBUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERBUTTON), interruptHandler, FALLING);

  oled.begin();
  oled.clear(PAGE);

  motor.setAcceleration(20000);
  motor.setMaxSpeed(10000);

  degPerPulse = 360.0 / (float)PPR;

}//END SETUP
float loopLastPosition = 0.0;
void loop() {

  float fWOmega = flyWheelOmega();
  
  if (speedChange) {
    buttonEnc.write(loopLastPosition * 24.0);
    while (speedChange) {
      motor.setSpeed(0);
      motor.runSpeed();
      motorSpeed = buttonEncReading() / 24.0;
      Serial.println(motorSpeed);
      Serial.println("");
      screenWriting(motorSpeed);
    }
  }
  loopLastPosition = motorSpeed;
  motor.setSpeed(motorSpeed);
  motor.runSpeed();

}//END LOOP

//---------------------------------------------------------------------------
/**
   Counts if the flywheel moved clockwise or counterclockwise
*/
long movementCounter = 0;

int countMovement(float omegaValue) {
  if (omegaValue > 0) {
    movementCounter++;
  } else if (omegaValue < 0) {
    movementCounter--;
  }
  return movementCounter;
}//END CountMovement

//---------------------------------------------------------------------------
/**
   Reads the encoder in the fly wheel and calculates the omega of the flywheel
   Returns the omega value of the flywheel
*/

long oldPosition = 999;
long oldTime = 0;
float omega = 1.0;

float flyWheelOmega() {

  newPosition = flyWheelEnc.read();

  //Should return the current angle of the flywheel
  fWOutput = newPosition * degPerPulse;

  if (newPosition != oldPosition) {
    //Finding the omega of the flywheel
    deltaPhi = degPerPulse * (newPosition - oldPosition);

    long newTime = flyWheelTimer;
    deltaTime = float(newTime - oldTime) / float(1000000.0);

    omega = deltaPhi / deltaTime;

    //Saves the new data
    oldTime = newTime;
    oldPosition = newPosition;
  }
  return omega;
}//END FlyWheelRead

//---------------------------------------------------------------------------
/**
   reads the encoder, limits it and maps the new value
   2200 is about the limit before the stepper starts slipping
*/
float buttonEncReading() {
  float encPosition = buttonEnc.read();
  //Serial.println(encPosition,8);
  noInterrupts();
  float maxPosition = 50000.0;
  float minPosition = 0.0;

  if (encPosition < minPosition) {
    encPosition = minPosition;
    //Forces the encoder to have a min value
    buttonEnc.write(minPosition);
  }
  if (encPosition > maxPosition) {
    encPosition = maxPosition;
    //Forces the encoder to have a max value
    buttonEnc.write(maxPosition);
  }

  //encPosition = map(encPosition, minPosition, maxPosition, 50, 500);
  interrupts();
  return encPosition;

}//END buttonEncReading

//---------------------------------------------------------------------------
/**
   Finds the RPM and Omega values
   Writes everything needed to the screen
*/
float newMotorOmega = 31.48;

bool newOmega = false;
float lastPosition = -999;

void screenWriting(float motSpeed) {

  if (motSpeed == lastPosition) {
    float rpm = ((motSpeed * 60.0) / 200.0);
    float omega = ((((rpm * 360.0) / 60.0) * M_PI) / 180.0);

    //Serial.println(rpm);
    //Serial.println("");
    //Serial.println(omega);

    oled.setFontType(1);
    oled.setCursor(0, 0);
    oled.print("RPM:");
    oled.setFontType(0);
    oled.setCursor(40, 4);
    oled.print(rpm, 3);
    //oled.print(motSpeed);
    oled.setFontType(1);
    oled.setCursor(0, 15);
    oled.print("Omega: ");
    oled.setCursor(59, 20);
    oled.setFontType(0);
    oled.print(omega,4);
    oled.display();
  }
  lastPosition = motSpeed;
}//END ScreenWriting


//---------------------------------------------------------------------------
/**
   Tells the interupt which function to call depending on the the times the button has been pushed
   Inlcudes a debounder to deal with the mechanical button
*/

unsigned long lastInterrupt = 0;

void interruptHandler() {
  unsigned long interrupt = millis();

  //If a interupt happens before 200 milliseconds, ignore it
  if (interrupt - lastInterrupt > 200) {
    speedChange = !speedChange;
    lastInterrupt = interrupt;
  }
}

//---------------------------------------------------------------------------
/**
   Moves the motor with the encoder to be able to set the motor to reset point
*/
int lastEncPosition = 0;

void motorReset() {
  Serial.println("Inside motorReset()");

  int encPosition = buttonEnc.read();
  //  int motorPosition = motor.getPosition();
  //  Serial.print(motorPosition);
  Serial.print("  ,  ");
  Serial.println(encPosition);
  Serial.println("");
  if (encPosition > lastEncPosition) {
    //    motorPosition += 100;
    // controller.move(motor);
    Serial.println("Return from greater then move");
  } else if (encPosition < lastEncPosition) {
    //motorPosition += -100;
    //controller.move(motor);
    Serial.println("Return from less then move");
  }
  lastEncPosition = encPosition;
}

