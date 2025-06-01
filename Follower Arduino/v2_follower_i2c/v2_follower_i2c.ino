// *** FOLLOWER ARDUINO CODE, ME327 Group 1, 2025 *** //
// This file is the code for the follower Arduino. The job of this file is to read the encoder of the
// motor (https://www.pololu.com/product/4881/resources) connected to the follower Arduino. It then 
// transmits this information in bytes across the Serial link. This approach was done because the 
// Hapkit has only two interrupt pins. Thus we cannot connect two motors (each with an encoder requiring
// two interrupt pins) to a single Hapkit. Therefore we use a second Hapkit to read the encoder of the motor
// connected to it. All the torque control of both motors and talking to processing is done from the master 
// Hapkit.

// #define DEBUG
// #define LEFT_FOLLOWER
#define RIGHT_FOLLOWER

#include <Encoder.h>
#include <Wire.h>

const int SERIAL_BAUD_RATE = 9600;
const int I2C_CLOCK        = 400000;
const int SPEED_THROTTLE   = 0;     // This was for a potential implementation of a delay in the void loop but is unused. 

#ifdef LEFT_FOLLOWER
const int I2C_ADDRESS = 8;
const uint8_t SRC_ID  = 0x01;  // left follower hapkit comms ID tag

#endif

#ifdef RIGHT_FOLLOWER
const int I2C_ADDRESS = 9;
const uint8_t SRC_ID  = 0x02;  // right follower hapkit comms ID tag

#endif

// Address of the master Hapkit Arduino on the I2C bus
const int MASTER_ADDRESS = 1;

// Define encoder pins.
const int ENCODER_PIN_1  = 2;
const int ENCODER_PIN_2  = 3;

// Create encoder object for theta5, using interrupt pins 2 and 3.
Encoder enc(ENCODER_PIN_1, ENCODER_PIN_2);

// Buffer the last read so the ISR can grab it quickly
volatile int16_t lastCount = 0;

// Function prototypes.
void sendCountsToMaster();

// This function runs once on startup.
void setup()
{
  // Configure Encoder pins
  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);

  // Set encoder to 0 on power on.
  enc.write(0);

  #ifdef DEBUG
  Serial.begin(SERIAL_BAUD_RATE);
  #endif

  Wire.begin(I2C_ADDRESS);
  Wire.setClock(I2C_CLOCK);
  Wire.onRequest(sendCountsToMaster);
}

// This function runs continuously.
void loop()
{
  lastCount = enc.read();

  #ifdef DEBUG
  Serial.print("Encoder Count: ");
  Serial.println(lastCount);
  #endif
}

// This function sends the constructed two-byte encoder count to the
// main arduino over the I2C bus.
void sendCountsToMaster()
{
  uint8_t lo = lowByte(lastCount);
  uint8_t hi = highByte(lastCount);

  // build packet [ID][LO][HI]
  uint8_t dataBytes[3] = {SRC_ID, lo, hi};

  // send to master
  Wire.write(dataBytes, sizeof(dataBytes));
}
