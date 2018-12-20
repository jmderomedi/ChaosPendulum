/*
   Physics Capstone Project
   Author: James Deromedi
   Mentor: Jonathon Newport
*/
//---------------------------------------------------------------------
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <SavLayFilter.h>
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
//---------------------------------------------------------------------
#define PIN_RESET 15
#define PIN_DC    5
#define PIN_CS    10
#define PIN_SCK   13
#define PIN_MOSI  11
//---------------------------------------------------------------------
AccelStepper motor(1, 17, 16);
Encoder buttonEnc(22, 23);
Encoder flyWheelEnc(7, 6);
TeensyView oled(PIN_RESET, PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI);
SavLayFilter sgFilterOmega = SavLayFilter();
SavLayFilter sgFilterAngle = SavLayFilter();
SavLayFilter sgFilterTheta = SavLayFilter();
//---------------------------------------------------------------------
elapsedMicros flyWheelTimer;
elapsedMicros driveTimer;

const int PPR = 4096;
const int ENCODERBUTTON = 18;

unsigned long oldTime = 0;
unsigned long newTime;
unsigned long lastInterrupt = 0;

float omega = 1.0;
float fWOutput = 0.0;
float degPerPulse = 0.0;
float motorSpeed = 50.0;
float newPosition = 0.0;
float loopLastPosition = 97.0;
float oldPosition = -9999;
float fWOmega = -9999;
float lastPosition = -999;
float motorOldPosition = 0.0;
float deltaPhi = 0.0;
float deltaTime = 0.0;

int dataCount = 0;
int motorPosition = 0;

bool speedChange = false;
//---------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  pinMode(ENCODERBUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERBUTTON), interruptHandler, FALLING);

  oled.begin();
  oled.clear(PAGE);
  motor.setAcceleration(3000);
  motor.setMaxSpeed(10000);

  degPerPulse = 360.0 / (float)PPR;
  Serial.println("CLEARDATA");
  Serial.println("LABEL,angle,omega,fangle,fomega");
}//END SETUP

//---------------------------------------------------------------------
void loop() {
  if (speedChange) {
    buttonPushLoop();
  }

  motorPosition = motor.currentPosition();   //Check if the flywheel is in a new position
  if (motorPosition != motorOldPosition) {
    newPosition = flyWheelEnc.read();
    dataCount++;
    fWOmega = calculateOmega();                //Calculates the flywheel omega
    motorOldPosition = motorPosition;         //Saves the current motor position
    oldPosition = newPosition;                //Saves the current flywheel positon
    printOutValues();
  }
  motor.setSpeed(motorSpeed);                 //Takes a step with the motor at the current speed
  motor.runSpeed();
}//END LOOP

//---------------------------------------------------------------------------
/**
   Reads the encoder in the fly wheel and calculates the omega of the flywheel
   Returns the omega value of the flywheel
*/
float calculateOmega() {
  newTime = flyWheelTimer;                  //Saves the current time
  fWOutput = (newPosition * degPerPulse) * M_PI / 180.0;      //Should return the current angle of the flywheel in rads
  deltaPhi = degPerPulse * (newPosition - oldPosition);
  deltaTime = float(newTime - oldTime) / float(1000000.0);
  omega = deltaPhi / deltaTime;
  oldTime = newTime;                          //Saves the new data
  oldPosition = newPosition;
  return omega;
}//END FlyWheelRead

//---------------------------------------------------------------------------
/**
   reads the encoder, limits it and maps the new value
   2200 is about the limit before the stepper starts slipping
*/
float buttonEncReading() {
  float encPosition = buttonEnc.read();
  noInterrupts();
  float maxPosition = 100000.0;
  float minPosition = 50.0;

  if (encPosition < minPosition) {     //Forces the encoder to have a min value
    encPosition = minPosition;
    buttonEnc.write(minPosition);
  }
  if (encPosition > maxPosition) {    //Forces the encoder to have a max value
    encPosition = maxPosition;
    buttonEnc.write(maxPosition);
  }
  interrupts();
  return encPosition;
}//END buttonEncReading

//---------------------------------------------------------------------------
/**
   Finds the RPM and Omega values
   Writes everything needed to the screen
*/
void screenWriting(float motSpeed) {
  if (motSpeed == lastPosition) {
    float rpm = ((motSpeed * 60.0) / 200.0);
    float omega = (((float(rpm * 360.0) / 60.0) * M_PI) / 180.0);  //Omega in radians

    oled.setFontType(1);
    oled.setCursor(0, 0);
    oled.print("RPM:");
    oled.setFontType(0);
    oled.setCursor(40, 4);
    oled.print(rpm, 3);
    oled.setFontType(1);
    oled.setCursor(0, 15);
    oled.print("Omega: ");
    oled.setCursor(59, 20);
    oled.setFontType(0);
    oled.print(omega, 4);
    oled.display();
  }
  lastPosition = motSpeed;

}//END ScreenWriting

//---------------------------------------------------------------------------
/**
   Tells the interupt which function to call depending on the the times the button has been pushed
   Inlcudes a debounder to deal with the mechanical button
*/
void interruptHandler() {
  unsigned long interrupt = millis();

  if (interrupt - lastInterrupt > 200) {         //If a interupt happens before 200 milliseconds, ignore it
    speedChange = !speedChange;
    lastInterrupt = interrupt;
  }
}//END interruptHandler()

//---------------------------------------------------------------------------
/**
   A function used to print out values to serial to be saved later into a notepad
*/
void printOutValues() {
  //Serial.print(driveTimer);
  //Serial.print(",");


  float filteredOmega = sgFilterOmega.smoothing(5, fWOmega, 0);
  float filteredAngle = sgFilterAngle.smoothing(5, fWOutput, 0);
  if ((dataCount % 200) == 47) {
    Serial.print("DATA,");
//    Serial.print(fWOutput, 8);
//    Serial.print(",");
//    Serial.print(fWOmega, 8);
//    Serial.print(",");
    Serial.print(filteredAngle, 8);
    Serial.print(",");
    Serial.println(filteredOmega, 8);

  }
  if (dataCount == 200) {
    dataCount = 0;
  }

  //Serial.print(",");
  //Serial.println(sgFilterTheta.smoothing(5, filteredOmega, 1), 8);
}

void resetVariables() {
  dataCount = 0;                           //Resets datacount since it is a new speed
  driveTimer = 0;                          //Resets the driveTimer to zero
  sgFilterOmega.resetValues();             //Resets the smoothing array for omega
  sgFilterAngle.resetValues();             //Resets the smoothing array for omega
  sgFilterTheta.resetValues();
}

void buttonPushLoop() {
  buttonEnc.write(loopLastPosition * 384);    //Times by 384 for more accurate dialing

  while (speedChange) {
    motor.setSpeed(0);
    motor.run();
    motorSpeed = buttonEncReading() / 384;    //Divide by 384 for more accurate dialing
    screenWriting(motorSpeed);
  }
  loopLastPosition = motorSpeed;           //Saves the new speed so the screen always shows the right number
  resetVariables();                       //Resets all important data when button is pushed
}

