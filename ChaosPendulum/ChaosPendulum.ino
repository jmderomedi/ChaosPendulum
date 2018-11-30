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
//---------------------------------------------------------------------
elapsedMicros flyWheelTimer;
elapsedMicros driveTimer;
//---------------------------------------------------------------------
const int PPR = 4096;
const int ENCODERBUTTON = 18;
//---------------------------------------------------------------------
unsigned long oldTime = 0;    //flyWheelOmega()
unsigned long newTime;        //flyWheelOmega()
unsigned long lastInterrupt = 0;
//---------------------------------------------------------------------
float omega = 1.0;            //flyWheelOmega()
float fWOutput = 0.0;         //flyWheelOmega()
float degPerPulse = 0.0;      //flyWheelOmega()
float motorSpeed = 100.0;     //loop()
float newPosition = 0.0;      //loop()/flyWheelOmega()
float loopLastPosition = 150; //loop()
float oldPosition = -9999;    //loop()/flyWheelOmega()
float fWOmega = -9999;        //loop()/flyWheelOmega()
float lastPosition = -999;    //loop()/screenWriting()
float motorOldPosition = 0.0; //loop()
float deltaPhi = 0.0;         //flyWheelOmega()
float deltaTime = 0.0;        //flyWheelOmega()
//---------------------------------------------------------------------
int dataCount = 0;            //loop()
int motorPosition = 0;        //loop()
//---------------------------------------------------------------------
bool speedChange = false;     //loop()/interruptHandler
//---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(ENCODERBUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERBUTTON), interruptHandler, FALLING);

  oled.begin();
  oled.clear(PAGE);

  motor.setAcceleration(10000);
  motor.setMaxSpeed(11000);
  degPerPulse = 360.0 / (float)PPR;
}//END SETUP

//---------------------------------------------------------------------
void loop() {
  if (speedChange) {
    buttonEnc.write(loopLastPosition);    //Times by 384 for more accurate dialing

    while (speedChange) {
      motor.setSpeed(0);
      motorSpeed = buttonEncReading();    //Divide by 384 for more accurate dialing
      screenWriting(motorSpeed);
    }
    loopLastPosition = motorSpeed;           //Saves the new speed so the screen always shows the right number
    dataCount = 0;                           //Resets datacount since it is a new speed
    driveTimer = 0;                          //Resets the driveTimer to zero
    sgFilterOmega.resetValues();             //Resets the smoothing array for omega
    sgFilterAngle.resetValues();             //Resets the smoothing array for omega
  }

  motorPosition = motor.currentPosition();   //Check if the flywheel is in a new position
  if (motorPosition != motorOldPosition) {
    newPosition = flyWheelEnc.read();
    dataCount++;

    if (motorPosition % 200 == 0) { // The motor has made a full rotation
      dataCount = 0;
    }
    
    if (dataCount == 50) {                    //Prints out the data at the same specific point
      testing();
    }

    newTime = flyWheelTimer;                  //Saves the current time
    fWOmega = flyWheelOmega();                //Calculates the flywheel omega
    motorOldPosition = motorPosition;         //Saves the current motor position
    oldPosition = newPosition;                //Saves the current flywheel position 
  }
  motor.setSpeed(motorSpeed);                 //Takes a step with the motor at the current speed
  motor.runSpeed();
}//END LOOP

//---------------------------------------------------------------------------
/**
   Reads the encoder in the fly wheel and calculates the omega of the flywheel
   Returns the omega value of the flywheel
*/
float flyWheelOmega() {
  fWOutput = (newPosition * degPerPulse);      //Should return the current angle of the flywheel
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
 * A function used to print out values to serial to be saved later into a notepad
 */
void testing() {
  Serial.print(driveTimer);                                       //Prints out the count for post processing in MatLab
  Serial.print(",");
  Serial.print(fWOmega, 8);
  Serial.print(",");
  Serial.print(fWOutput, 8);
  Serial.print(",");
  Serial.print(sgFilterOmega.quadCubicSmooth(5, fWOmega), 8);     //Prints out the angluar frequency of the flywheel
  Serial.print(",");
  Serial.println(sgFilterAngle.quadCubicSmooth(5, fWOutput), 8);   //Prints out the angle of the flywheel
}
