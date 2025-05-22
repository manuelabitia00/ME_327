// Created:  ME327 Group 1, 2024
// Modified: ME327 Group 1, 2025

// This file is the code for the second (follower) arduino. This code 
// allows the second arduino to read the value of the encoder of the motor
// it's connected to (https://www.pololu.com/product/4881/resources)
// and send this information to the primary arduino.
// We are using two arduinos because the Uno (Hapkit) has only two interrupt
// pins and we need four to connect two motors.

// FINDINGS FROM TRANSMISSION TESTING
// -- Follower transmission (i.e. loop) speed MUST be SLOWER than leader, or can get out of sync
// -- Ensure this by increasing SPEED_THROTTLE above
// -- Loop speed will grow, so may have to increase SPEED_THROTTLE
// -- Note that going too slow doesn't hurt final accuracy (WILL GET FINAL ANGLE RIGHT NO MATTER WHAT)
// -- But does increase delay between user motion and response of haptic device

#include <Encoder.h>

const int BAUD_RATE      = 115200;
const int SPEED_THROTTLE = 10;

// Create encoder object for theta5, using interrupt pins 2 and 3.
Encoder encT5(2,3);

// Create new type BinaryIntUnion for sending two-byte encoder counts.
typedef union {
  int integer;
  byte binary[2];
} BinaryIntUnion;

// Function prototypes.
void sendRemoteArduino(int encoderCount);

void setup()
{
  // Configure Encoder pins
  pinMode(2, INPUT);
  pinMode(3, INPUT);

  // Initialize position. Assume starting at theta5 = 0 deg, may have to 
  // calibrate for correlation with a mechanical hard stop to ensure the 
  // number is correct on power on.
  int countsT5 = 0;
  encT5.write(0);
  
  Serial.begin(BAUD_RATE);
}

void loop()
{
  int countsT5 = encT5.read();
  sendRemoteArduino(countsT5);
  delay(SPEED_THROTTLE);
}

// This function sends the constructed two-byte encoder count to the 
// main arduino over the serial monitor. 
void sendRemoteArduino(int countsT5)
{ 
  // Declare unionCounts5 - a more efficient method of storing and 
  // transmitting our information.
  BinaryIntUnion unionCounts5;
  
  // Check that we have space in the serial buffer to write
  if (Serial.availableForWrite() > sizeof(int))
  {
    // Save space by using a integer representation
    unionCounts5.integer = countsT5;
    
    // write the integer to serial
    Serial.write(unionCounts5.binary, 2);
    Serial.flush();
  }
}
