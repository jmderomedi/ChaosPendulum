#define ENCODER_OPTIMIZE_INTERRUPTS

//#include "SavLayFilter.h"
#include <StepControl.h>
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

Stepper motor(16, 17);
Encoder buttonEnc(23, 22);
Encoder flyWheelEnc(5, 6);
TeensyView oled(PIN_RESET, PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI);

//SavLayFilter sgFilter();
StepControl<> controller;

elapsedMillis timeElapsed;
elapsedMicros flyWheelTimer;

const int PPR = 4096; //Pulses Per Revolution
const int ENCODERBUTTON = 18;

float degPerPulse = 0.0;
int motorSpeed = 0;

void setup() {
  Serial.begin(128000);
  pinMode(ENCODERBUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERBUTTON), motorSpeedChange, FALLING);

  oled.begin();
  oled.clear(ALL);

  motor.setAcceleration(300000);
  motor.setMaxSpeed(1102);
  controller.rotateAsync(motor);

  degPerPulse = 360.0 / (float)PPR;

  //  Testing the time the motor takes to complete 8 rotations
  //  motor.setMaxSpeed(1600);
  //  motor.setTargetRel(1600);
  //  timeElapsed = 0;
  //  controller.move(motor);
  //  Serial.println(timeElapsed);
  //  Serial.println(motor.getPosition());

}//END SETUP

void loop() {
  oled.clear(PAGE);

  float fWOmega = flyWheelOmega();
  motorSpeed = buttonEncReading();
  screenWriting(motorSpeed);

  Serial.println(fWOmega);

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
}//END CountMovement

//---------------------------------------------------------------------------
/**
   Reads the encoder in the fly wheel and calculates the omega of the flywheel
   Returns the omega value of the flywheel
*/
long oldPosition = 0;
long newPosition = 0;
long oldTime = 0;
long newTime = 0;

float flyWheelOmega() {
  float deltaPhi = 0.0;
  float deltaTime = 0.0;
  float omega = 0.0;
  float phi = 0.0;
  //float t = 0.0;

  newPosition = flyWheelEnc.read();

  if (newPosition != oldPosition) {
    newTime = flyWheelTimer;

    //Finding the omega of the flywheel
    deltaPhi = degPerPulse * (float)(newPosition - oldPosition);
    deltaTime = (float)(newTime - oldTime) / 1000000.0;
    omega = deltaPhi / deltaTime;

    //phi = degPerPulse * (float)oldPosition + deltaPhi / 2.0; //The current angle of the plate
    //t = (float)oldTime / 1000000.0 + deltaTime / 2.0; //The time of reading in seconds

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
int buttonEncReading() {
  int encPosition = buttonEnc.read();
  int maxPosition = 200;
  int minPosition = -200;

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
  return encPosition = map(encPosition, minPosition, maxPosition, 5, 2000);
}//END buttonEncReading

//---------------------------------------------------------------------------
/**
   Finds the RPM and Omega values
   Writes everything needed to the screen
*/
float newMotorOmega = 31.48;
bool speedChange = false;
bool newOmega = false;

void screenWriting(int motSpeed) {
  //Finds the RPM of the motor for display
  float rpm = (((motSpeed * 60) * 1.8) / 360);
  //Finds the rotational frequency of the motor
  float omega = (((rpm / 60) * 360) * M_PI) / 180;

  //Allows the LCD to only change the omega value on button push
  if (speedChange) {
    newMotorOmega = omega;
    speedChange = false;
    newOmega = true;
  } else {
    omega = newMotorOmega;
    newOmega = false;
  }

  oled.setFontType(1);
  oled.setCursor(0, 0);
  oled.print("RPM:");
  oled.setFontType(0);
  oled.setCursor(40, 4);
  oled.print(rpm);
  oled.setFontType(1);
  oled.setCursor(0, 15);
  oled.print("Omega: ");
  oled.setCursor(59, 20);
  oled.setFontType(0);
  if (newOmega) {
    oled.print(newMotorOmega);
  } else {
    oled.print(omega);
  }
  oled.display();
}//END ScreenWriting

//---------------------------------------------------------------------------
/**
   Called when the interupt is triggered from the encoder button
   Stops the motor and waits for return
   Sets the new speed and starts with immeditate return
*/
void motorSpeedChange() {
  speedChange = true;
  controller.stop();
  motor.setMaxSpeed(motorSpeed);
  controller.rotateAsync(motor);
}//END MotorSpeedChange

