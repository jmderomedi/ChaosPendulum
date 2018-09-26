#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"
#include "SavLayFilter.h"



const int PPR = 4096; //Pulses Per Revolution
const int SIGNALA = 6;
const int SIGNALB = 5;
float degPerPulse;

Encoder myEnc(SIGNALB, SIGNALA);  //Pins can be changed to any input pin for teensy 3.2 except ones with leds attached
SavLayFilter sgFilter();

void setup() {
  Serial.begin(128000);
  degPerPulse = 360.0 / (float)PPR; // = 0.087890625 roughly


  Serial.println("LABEL, omega, smoothedOmega");
  Serial.println("RESETTIMER");
}

long oldPosition = 0;
long newPosition = 0;
long oldTime = 0;
long newTime = 0;
float alpha;
float omega;
float phi;
float t;
elapsedMicros timer1;
long movementCounter = 0;
float movingDataArray [9];
float smoothedData = 0.0;
int populateCount = 0;
int smoothedCount = 0;

void loop() {

  newPosition = myEnc.read();   //Reading the encoder

  if (newPosition != oldPosition) {   //If the disk is in a new position
    newTime = timer1; //Saving the time of "reading"
    long deltaPosition = newPosition - oldPosition; //Change of how much the disk has moved
    float deltaPhi = degPerPulse * (float)deltaPosition; //Change in angle of the plate since last pulse/"reading"
    long deltaMicros = newTime - oldTime; //Change in time of "reading" in micros
    float deltaTime = (float)deltaMicros / 1000000.0; //Converting change in time into seconds
    omega = deltaPhi / deltaTime; //Rate of change of the angle of the plate
    phi = degPerPulse * (float)oldPosition + deltaPhi / 2.0; //The current angle of the plate
    t = (float)oldTime / 1000000.0 + deltaTime / 2.0; //The time of reading in seconds

    oldTime = newTime;  //Saving the the time of the "reading"
    oldPosition = newPosition;  //Saving the position of the "reading"

    if (omega > 0) {
      movementCounter++;
    } else if (omega < 0) {
      movementCounter--;
    }

      smoothedData = sgFilter.quadCubicSmooth(5, omega);
      //smoothedDataArray[smoothedCount] = sg_filter(omega);
      Serial.print("DATA, Omega, SmoothedOmega");
      Serial.print(omega);
      Serial.print(" , ");
      Serial.println(smoothedData);
  }
}

