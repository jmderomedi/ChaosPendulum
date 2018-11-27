#include <Wire.h>
#include <AccelStepper.h>

AccelStepper motor(1, 17, 16);

void setup() {
  Serial.begin(115200);
  motor.setAcceleration(10000);
  motor.setMaxSpeed(11000);


}

void loop() {

  motor.setSpeed(400);
  motor.runSpeed();
  //motor.move(200);
  //motor.runToPosition();
  //delay(1000);
}
