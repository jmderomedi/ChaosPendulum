#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>



const int PPR = 4096; //Pulses Per Revolution
const int SIGNALA = 6;
const int SIGNALB = 5;
float degPerPulse;

Encoder myEnc(SIGNALB, SIGNALA);  //Pins can be changed to any input pin for teensy 3.2 except ones with leds attached

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
float smoothedDataArray [13999];
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

    if (populateCount == 8){
      //Applies sg filter and moves the data over by one
      //Global array means it can be accessed in the function
      //TODO: Change the array from global into pointers
      
      //smoothedDataArray[smoothedCount] = sg_filter(omega);
      Serial.print("DATA, count ");
      Serial.print(smoothedCount);
      Serial.print(",");
      Serial.print(omega);
      Serial.print(",");
      Serial.println(sg_filter(omega));
      smoothedCount++;
    } else {
      movingDataArray[populateCount] = omega; //Populates the first array with data
      populateCount++;
      //Serial.println(movingDataArray[populateCount] + " populatecount"); 
    }
    
    //    Serial.print(phi); 
    //    Serial.print("\t\t");
    //    Serial.print(omega);  //The rate of change of the plate
    //    Serial.print("\t\t");
    //    Serial.println(movementCounter/4);
  }
}

/**
   I am hardcoding the the values of the windowarray in, this will be to debug
   When the library is created than I will make it more variable
   I have the windowArray global currently, this way it can be changed within the function
*/
float sg_filter(float currentInput) {
  int sg_coefficent[] = { -21, 14, 39, 54, 59, 54, 39, 14, -21}; //Values from SG article
  float normalization_factor = 231;  //Value from SG article
  int windowLength = 9;
  int windowMidPoint = (9 - 1) / 2;
  float sum = 0;

  //Move through the array and summing the values * coeffcient
  for (int i = 0; i < windowMidPoint; i++) {
    sum += (movingDataArray[i] + movingDataArray[(windowLength - 1) - i]) * sg_coefficent[i];
  }
  sum += movingDataArray[windowMidPoint] * sg_coefficent[windowMidPoint];
  float smoothedData = sum / normalization_factor;

  //Move array over by one and add in new current value
  //Need to test if before or after summing is important
  for (int i = 1; i < windowLength; i++) {
    movingDataArray[i - 1] = movingDataArray[i];
  }
  movingDataArray[windowLength - 1] = currentInput;
  //returns the new value calculated in the smoothing process
  //Since can not return an array in Arduino
  return smoothedData;
}

