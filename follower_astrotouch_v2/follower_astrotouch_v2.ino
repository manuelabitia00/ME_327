// *** FOLLOWER ARDUINO CODE, ME327 Group 1, 2025 *** //
// This file is the code for the follower Arduino. The job of this file is to read the encoder of the
// motor connected to the follower Arduino. It then transmits this information via I2C to the master.

// #define DEBUG
// #define LEFT_FOLLOWER
#define RIGHT_FOLLOWER

#include <Encoder.h>
#include <Wire.h>

const int SERIAL_BAUD_RATE = 115200;  // FIXED: Match leader for debugging consistency
const int I2C_CLOCK        = 400000;  // This is correct - matches leader now

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

// Create encoder object using interrupt pins 2 and 3.
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
  Serial.println("AstroTouch Follower Starting...");
  #ifdef LEFT_FOLLOWER
  Serial.println("LEFT_FOLLOWER at address 8");
  #endif
  #ifdef RIGHT_FOLLOWER
  Serial.println("RIGHT_FOLLOWER at address 9");
  #endif
  #endif

  Wire.begin(I2C_ADDRESS);
  Wire.setClock(I2C_CLOCK);
  Wire.onRequest(sendCountsToMaster);
  
  #ifdef DEBUG
  Serial.println("I2C initialized, ready for requests");
  #endif
}

// This function runs continuously.
void loop()
{
  // Read encoder and store in volatile variable for ISR access
  lastCount = enc.read();

  #ifdef DEBUG
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 100) { // Debug every 100ms
    Serial.print("Encoder Count: ");
    Serial.println(lastCount);
    lastDebugTime = millis();
  }
  #endif
  
  // Small delay to prevent overwhelming the system
  delay(1);
}

// This function sends the constructed two-byte encoder count to the
// main arduino over the I2C bus when requested.
void sendCountsToMaster()
{
  // Capture the count atomically
  int16_t currentCount = lastCount;
  
  uint8_t lo = lowByte(currentCount);
  uint8_t hi = highByte(currentCount);

  // build packet [ID][LO][HI]
  uint8_t dataBytes[3] = {SRC_ID, lo, hi};

  // send to master
  Wire.write(dataBytes, sizeof(dataBytes));
  
  #ifdef DEBUG
  Serial.print("Sent to master: ID=0x");
  Serial.print(SRC_ID, HEX);
  Serial.print(" Count=");
  Serial.println(currentCount);
  #endif
}