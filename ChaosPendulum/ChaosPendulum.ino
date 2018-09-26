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

#define PIN_RESET 15
#define PIN_DC    5
#define PIN_CS    10
#define PIN_SCK   13
#define PIN_MOSI  11

Stepper motor(16, 17);
Encoder myEnc(23, 22);
TeensyView oled(PIN_RESET, PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI);

StepControl<> controller;

//elapsedMillis timeElapsed;
const int ENCODERBUTTON = 18;
int encoderPos = 0;

void setup() {
  //Serial.begin(9600);
  pinMode(ENCODERBUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERBUTTON), motorSpeedChange, FALLING);

  oled.begin();
  oled.clear(ALL);

  motor.setAcceleration(300000);
  motor.setMaxSpeed(2000);

//For testing the time to complete 8 rotatations
//  motor.setTargetRel(1600);
//  timeElapsed = 0;
//  controller.move(motor);
//  Serial.println(timeElapsed);
//  Serial.println(motor.getPosition());
  
  //Starts the motor for the first time at the mean value of the encoder
  controller.rotateAsync(motor);
}

void loop() {
  oled.clear(PAGE);

  encoderReading();
  screenWriting();
  
}//END LOOP

/**
 * reads the encoder, limits it and maps the new value
 * 2000 is about the limit before the stepper starts slipping
 */
void encoderReading() {
  encoderPos = myEnc.read();
  
  if (encoderPos < -200) {
    encoderPos = -200;
    myEnc.write(-200);
  }
  if (encoderPos > 200) {
    encoderPos = 200;
    myEnc.write(200);
  }
  encoderPos = map(encoderPos, -200, 200, 5, 2000);
}//END EncoderReading

/**
 * Writes everything needed to the screen
 */
void screenWriting() {
  oled.setFontType(1);
  oled.setCursor(0, 0);
  oled.print("'Speed': ");
  oled.print(encoderPos);
  oled.display();
}//END ScreenWriting

/**
 * Called when the interupt is triggered from the encoder button
 * Must stop the motor then set the speed and restart the motor
 */
void motorSpeedChange() {
  controller.stop();
  motor.setMaxSpeed(encoderPos);
  controller.rotateAsync(motor);
}//END MotorSpeedChange

