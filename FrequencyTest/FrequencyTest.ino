#define ENCODER_OPTIMIZE_INTERRUPTS
#include <elapsedMillis.h>
#include <Encoder.h>

Encoder flyWheelEnc(7, 6);

elapsedMillis flyWheelTimer;

const int oscillations = 6;
float newPosition = 999.9;
float oldPosition = 999.9;
float currentPos = 0.0;
int count = 0;
bool startFlag = false;
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Frequency Testing");
  newPosition = flyWheelEnc.read();
  currentPos = newPosition;
}

void loop() {
  newPosition = flyWheelEnc.read();

  if (newPosition > currentPos && !startFlag) {
    Serial.println("Start!");
    startFlag = true;
    flyWheelTimer = 0;
  }

  if (newPosition != currentPos && currentPos != oldPosition && startFlag) {
    if (newPosition > currentPos && currentPos < oldPosition) {
      Serial.println("newPosition < oldPosition");
      count++;
    }
    oldPosition = currentPos;
    currentPos = newPosition;
  }
//
//  Serial.println(oldPosition);
//    Serial.println(currentPos);
//      Serial.println(newPosition);
  if (count == oscillations) {
    Serial.println(flyWheelTimer);
    Serial.println(count);
    Serial.println("");
    count++;
  }

}
