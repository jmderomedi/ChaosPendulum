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
Encoder myEnc(23, 22);
TeensyView oled(PIN_RESET, PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI);
elapsedMillis timeElapsed;
StepControl<> controller;

const int ENCODERBUTTON = 18;
int motorSpeed = 0;
float newMotorOmega = 31.48;
bool speedChange = false;
bool newOmega = false;

void setup() {
  Serial.begin(9600);
  pinMode(ENCODERBUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERBUTTON), motorSpeedChange, FALLING);

  oled.begin();
  oled.clear(ALL);

  motor.setAcceleration(300000);
  motor.setMaxSpeed(1102);
  //Starts the motor for the first time at the mean value of the encoder
  controller.rotateAsync(motor);

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

  motorSpeed = encoderReading();
  Serial.println(motorSpeed);
  screenWriting(motorSpeed);

}//END LOOP

/**
   reads the encoder, limits it and maps the new value
   2200 is about the limit before the stepper starts slipping
*/
int encoderReading() {
  int encPosition = myEnc.read();
  int maxPosition = 200;
  int minPosition = -200;

  if (encPosition < minPosition) {
    encPosition = minPosition;
    //Forces the encoder to have a min value
    myEnc.write(minPosition);
  }
  if (encPosition > maxPosition) {
    encPosition = maxPosition;
    //Forces the encoder to have a max value
    myEnc.write(maxPosition);
  }
  return encPosition = map(encPosition, minPosition, maxPosition, 5, 2000);
}//END EncoderReading

/**
   Writes everything needed to the screen
*/
void screenWriting(int motSpeed) {
  //Converts from steps per second to rotations per minute
  float rpm = (((motSpeed * 60) * 1.8) / 360);
  float omega = (((rpm / 60) * 360) * M_PI) / 180;

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
  oled.setCursor(40,4);
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

