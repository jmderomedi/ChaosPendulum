//#define ENCODER_OPTIMIZE_INTERRUPTS

//#include <SavLayFilter.h>
#include <StepControl.h>
#include <elapsedMillis.h>
#include <font5x7.h>
#include <font8x16.h>
#include <fontlargenumber.h>
#include <sevenSegment.h>
#include <TeensyView.h>
#include <Encoder.h>
#include <Wire.h>
//#include <AccelStepper.h>
#include <math.h>

#define PIN_RESET 15
#define PIN_DC    5
#define PIN_CS    10
#define PIN_SCK   13
#define PIN_MOSI  11

Stepper motor(17, 16);
Encoder buttonEnc(22, 23);
Encoder flyWheelEnc(7, 6);
TeensyView oled(PIN_RESET, PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI);


//SavLayFilter sgFilter();
StepControl<> controller;

elapsedMillis timeElapsed;
elapsedMicros flyWheelTimer;

const int PPR = 4096;
const int ENCODERBUTTON = 18;

float fWOutput = 0.0;
float degPerPulse = 0.0;
int motorSpeed = 0;

float deltaPhi = 0.0;
float deltaTime = 0.0;

void setup() {
  Serial.begin(19200);
  pinMode(ENCODERBUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERBUTTON), interuptHandler, FALLING);

  oled.begin();
  oled.clear(PAGE);

  motor.setAcceleration(300000);
  motor.setMaxSpeed(248);
  controller.rotateAsync(motor);

  degPerPulse = 360.0 / (float)PPR;

  //  Testing the time the motor takes to complete 8 rotations
  //  motor.setMaxSpeed(1600);
  //  motor.setTargetRel(1600);
  //  timeElapsed = 0;
  //  controller.move(motor);
  //  Serial.println(timeElapsed);
  //  Serial.println(motor.getPosition());
  //Serial.println("CLEARDATA");
  //Serial.println("LABEL,count");

}//END SETUP

long countCount = 0;
void loop() {
  //oled.clear(PAGE);

  float fWOmega = flyWheelOmega();
  motorSpeed = buttonEncReading();
  screenWriting(motorSpeed);

  countCount++;

  //Serial.print("DATA,,");
  //Serial.print(countCount);
  //Serial.print("  ,  ");
  Serial.print(fWOutput);
  Serial.print(",");
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

  float newPosition = flyWheelEnc.read();
  //For printing position of flywheel

  //Should return the current angle of the flywheel
  fWOutput = newPosition * degPerPulse;

  if (newPosition != oldPosition) {
    //Finding the omega of the flywheel
    deltaPhi = degPerPulse * (newPosition - oldPosition);

    long newTime = flyWheelTimer;
    deltaTime = float(newTime - oldTime) / float(1000000.0);

    omega = deltaPhi / deltaTime;

    //    Serial.print(deltaTime); Serial.print("    ");
    //    Serial.print(deltaPhi); Serial.print("    ");
    //    Serial.println(omega);
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

  noInterrupts();
  int maxPosition = 300;
  int minPosition = -300;

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

  encPosition = map(encPosition, minPosition, maxPosition, 120, 400);
  interrupts();
  return encPosition;

}//END buttonEncReading

//---------------------------------------------------------------------------
/**
   Finds the RPM and Omega values
   Writes everything needed to the screen
*/
float newMotorOmega = 31.48;
bool speedChange = false;
bool newOmega = false;
int lastPosition = -999;

void screenWriting(int motSpeed) {

  if (motSpeed == lastPosition) {
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
  }
  lastPosition = motSpeed;
}//END ScreenWriting

//---------------------------------------------------------------------------
/**
   Sets the new speed and starts with immeditate return
*/
void motorSpeedChange() {
  Serial.println("Inside motorReset()");
  controller.rotateAsync(motor);
}//END MotorSpeedChange

//---------------------------------------------------------------------------
/**
   Tells the interupt which function to call depending on the the times the button has been pushed
   Inlcudes a debounder to deal with the mechanical button
*/
bool oneButtonClick = false;
unsigned long lastInterupt = 0;

void interuptHandler() {
  unsigned long interupt = millis();

  //If a interupt happens before 200 milliseconds, ignore it
  if (interupt - lastInterupt > 200) {
    Serial.println("Inside interuptHandler");
    speedChange = true;
    controller.stop();
    motor.setMaxSpeed(motorSpeed);
    //oneButtonClick = !oneButtonClick;

    if (oneButtonClick == true) {
      motorReset();
    } else {
      motorSpeedChange();
    }
    
    lastInterupt = interupt;
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
  int motorPosition = motor.getPosition();
  Serial.print(motorPosition);
  Serial.print("  ,  ");
  Serial.println(encPosition);
  Serial.println("");
  if (encPosition > lastEncPosition) {
    motorPosition += 100;
    controller.move(motor);
    Serial.println("Return from greater then move");
  } else if (encPosition < lastEncPosition) {
    motorPosition += -100;
    controller.move(motor);
    Serial.println("Return from less then move");
  }
  lastEncPosition = encPosition;
}

