// *** FOLLOWER ARDUINO CODE, ME327 Group 1, 2025 *** //
// This file is the code for the follower Arduino. The job of this file is to read the encoder of the
// motor (https://www.pololu.com/product/4881/resources) connected to the follower Arduino. It then 
// transmits this information in bytes across the Serial link. This approach was done because the 
// Hapkit has only two interrupt pins. Thus we cannot connect two motors (each with an encoder requiring
// two interrupt pins) to a single Hapkit. Therefore we use a second Hapkit to read the encoder of the motor
// connected to it. All the torque control of both motors and talking to processing is done from the master 
// Hapkit.

#define DEBUG
// #define LEFT_FOLLOWER
#define RIGHT_FOLLOWER

#include <Encoder.h>

const int BAUD_RATE      = 9600;
const int SPEED_THROTTLE = 0;     // This was for a potential implementation of a delay in the void loop but is unused. 

// Packet framing and identification
const uint8_t START_BYTE = 0xAA;  // packet start marker

#ifdef LEFT_FOLLOWER
const uint8_t SRC_ID     = 0x01;  // left follower hapkit comms ID tag
#endif

#ifdef RIGHT_FOLLOWER
const uint8_t SRC_ID     = 0x02;  // right follower hapkit comms ID tag
#endif
// Define encoder pins.
const int ENCODER_PIN_1  = 2;
const int ENCODER_PIN_2  = 3;

// Create encoder object for theta5, using interrupt pins 2 and 3.
Encoder enc(ENCODER_PIN_1, ENCODER_PIN_2);

// Create new type BinaryIntUnion for sending two-uint8_t encoder counts.
typedef union {
  int integer;
  uint8_t binary[2];
} BinaryIntUnion;

// Function prototypes.
void sendCountsToMasterArduino(int counts);

// This function runs once on startup.
void setup()
{
  // Configure Encoder pins
  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);

  // Set encoder to 0 on power on.
  enc.write(0);
  Serial.begin(BAUD_RATE);
}

// This function runs continuously.
void loop()
{
  int counts = enc.read();

  #ifdef DEBUG
  Serial.print("Encoder Count: ");
  Serial.println(counts);
  #endif
  
  sendCountsToMasterArduino(counts);
  delay(SPEED_THROTTLE);
}

// This function sends the constructed two-byte encoder count to the 
// main arduino over the serial monitor. 
void sendCountsToMasterArduino(int counts)
{ 
  // Declare unionCounts5 - a more efficient method of storing and 
  // transmitting our information.
  BinaryIntUnion unionCounts5;

  // Save space by using a integer representation
  unionCounts5.integer = counts;

  // Prepare packet: start, ID, data[2], checksum
  uint8_t header[2]     = {START_BYTE, SRC_ID};
  uint8_t dataBytes[2]  = {unionCounts5.binary[0], unionCounts5.binary[1]};
  
  // Create the checksum using bitwise XOR. This is used to ensure our information
  // got sent across correctly to the master Arduino.
  uint8_t checksum      = SRC_ID ^ dataBytes[0] ^ dataBytes[1];
  
  // Check that we have space in the serial buffer to write
  if (Serial.availableForWrite() >= (sizeof(header) + sizeof(dataBytes) + 1))
  {
    Serial.write(header, sizeof(header));
    Serial.write(dataBytes, sizeof(dataBytes));
    Serial.write(&checksum, 1);
    Serial.flush();

    #ifdef DEBUG
    Serial.println();
    Serial.print("TX: ");
    // print header
    for (uint8_t i = 0; i < 2; i++) {
      Serial.print("0x");
      if (header[i] < 0x10) Serial.print('0');
      Serial.print(header[i], HEX);
      Serial.print(' ');
    }
    // print data bytes
    for (uint8_t i = 0; i < 2; i++) {
      Serial.print("0x");
      if (dataBytes[i] < 0x10) Serial.print('0');
      Serial.print(dataBytes[i], HEX);
      Serial.print(' ');
    }
    // print checksum
    Serial.print("0x");
    if (checksum < 0x10) Serial.print('0');
    Serial.print(checksum, HEX);
    Serial.println();
    #endif
  }
}
