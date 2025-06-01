// *** MASTER ARDUINO CODE, ME327 Group 1, 2025 *** //
// TODO Throughout this code, left encoder is referring to the encoder that is on 
// the left when looking at the back face of the pantograph.  

// Make sure debug is off otherwise it becomes extremely slow due to the processes that
// are being run!!
// #define DEBUG

#include <Encoder.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>

// Define communication constants.

const uint32_t I2C_BAUD_RATE          = 9600;
const uint32_t SERIAL_BAUD_RATE       = 9600;
const uint8_t  MASTER_ADDRESS         = 1;
const uint8_t  LEFT_FOLLOWER_ADDRESS  = 8;  // The id tag to identify comms from itsy bitsy reading the left encoder
const uint8_t  RIGHT_FOLLOWER_ADDRESS = 9;  // The id tag we identify comms from itsy bitsy reading the right encoder

// Define pins
const int MOTOR_1_PWM_PIN      = 5;
const int MOTOR_1_DIR_PIN      = 8;
const int MOTOR_5_PWM_PIN      = 6;
const int MOTOR_5_DIR_PIN      = 7;

// Define constants for the position calculation, many of these numbers were
// chosen because they were used (and tested) by ME327 Group 1 2024. 
const double COUNTS_PER_REV  = 48.0; 
const double RADIANS_PER_REV = 2.0 * PI;
const double GEAR_RATIO      = 4.4;
const double PULLEY_RADIUS   = 5.1;   // mm
const double SECTOR_RADIUS   = 75.0;  // mm
const double LINK_1_LENGTH   = 127.0; // 
const double LINK_2_LENGTH   = 152.4; // 
const double LINK_3_LENGTH   = 152.4; // Same as link 2
const double LINK_4_LENGTH   = 127.0; // Same as link 1
const double PIVOT_OFFSET    = 0.0;  // TODO This is a5 in the diagram but equals 0

const double MECHANICAL_LIMIT_THETA_DEG  = 15.0; // TODO This number needs tuning - it is currently an estimate. but we can fix it will scaling
const double THETA_1_START               = MECHANICAL_LIMIT_THETA_DEG * PI/180.0;
const double THETA_5_START               = (180.0 - MECHANICAL_LIMIT_THETA_DEG) * PI/180.0;

// Create a struct for the position of the cursor
struct CursorPos
{
  double x;
  double y;
};

// Create a struct to hold the forces for both motors.
struct MotorForces
{
  float f1;
  float f2;
};

// Declare function prototypes
int         readFollowerEncoder(int followerAddress, int lastValidCount);
CursorPos   calculateXYPosition(int countsT1, int countsT5);
void        sendPosToProcessing(CursorPos pos);
MotorForces readForcesFromProcessing();
void        setMotorForces(const MotorForces &forces);

// This function runs once on startup.
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  delay(100);
  #ifdef DEBUG
  Serial.println("Serial started");
  #endif

  Wire.begin(MASTER_ADDRESS);
  Wire.setClock(I2C_BAUD_RATE);
}

// This function runs continuously.
void loop()
{
  static MotorForces motorForces = {0.0, 0.0};
  static int lastValidCountT1 = 0;
  static int lastValidCountT5 = 0;

  lastValidCountT1 = -readFollowerEncoder(RIGHT_FOLLOWER_ADDRESS, lastValidCountT1);
  lastValidCountT5 =  readFollowerEncoder(LEFT_FOLLOWER_ADDRESS,  lastValidCountT5);

  #ifdef DEBUG
  Serial.print("Raw T1: "); Serial.println(lastValidCountT1);
  Serial.print("Raw T5: "); Serial.println(lastValidCountT5);
  #endif

  CursorPos pos = calculateXYPosition(lastValidCountT1, lastValidCountT5);

  sendPosToProcessing(pos);

  // motorForces = readForcesFromProcessing();
  // setMotorForces(motorForces);
}

// This function reads the encoder counts from the follower Arduinos over the I2C bus. 
// We pass lastValidCount by reference to ensure it gets changed in the loop and so 
// we don't have to make lastvalidcount = count
int readFollowerEncoder(int followerAddress, int lastValidCount)
{
  if (Wire.requestFrom(followerAddress, 3) == 3)
  {
    uint8_t srcID = Wire.read();
    uint8_t loByte = Wire.read();
    uint8_t hiByte = Wire.read();

    #ifdef DEBUG
    Serial.print("Received: 0x");
    if (srcID < 0x10) Serial.print('0');
    Serial.print(srcID, HEX);
    Serial.print(" 0x");
    if (loByte < 0x10) Serial.print('0');
    Serial.print(loByte, HEX);
    Serial.print(" 0x");
    if (hiByte < 0x10) Serial.print('0');
    Serial.println(hiByte, HEX);
    #endif

    // update the count
    lastValidCount = (int16_t)((hiByte << 8) | loByte);
    return lastValidCount;
  }
  else
  {
    #ifdef DEBUG
    Serial.println("I2C read failed");
    #endif
    return lastValidCount; // Return the last known valid count
  }
}

// This is a helper for calculateXYPosition. It converts the encoder counts
// to motor shaft revolutions in radians.
double countsToMotorRad(int encoderCounts)
{
  return (encoderCounts / COUNTS_PER_REV / GEAR_RATIO) * RADIANS_PER_REV;
}

// This function calculates and returns the current position of the cursor based
// on the current encoder counts.
CursorPos calculateXYPosition(int countsT1, int countsT5)
{
  CursorPos pos;
  
  // Get the motor shaft angles in radians
  double motorTheta1Rad = countsToMotorRad(countsT1);
  double motorTheta5Rad = countsToMotorRad(countsT5);
  
  // Get the link pivot angles
  double theta1Rad = THETA_1_START - (PULLEY_RADIUS / SECTOR_RADIUS) * motorTheta1Rad;
  double theta5Rad = THETA_5_START - (PULLEY_RADIUS / SECTOR_RADIUS) * motorTheta5Rad;

  // Serial.print("theta1deg: ");
  // Serial.print(theta1Rad * 180/PI);
  // Serial.print("theta5deg: ");
  // Serial.println(theta5Rad * 180/PI);
  
  // Compute the positions of the two intermediate link pivots (P2 and P4)
  // Both xs negative because x is pointing right with the origin at the central pivot.
  // ys are positive because positive ys are pointing down.
  double P2x = -LINK_1_LENGTH * cos(theta1Rad);
  double P2y =  LINK_1_LENGTH * sin(theta1Rad);
  double P4x = -LINK_4_LENGTH * cos(theta5Rad) + PIVOT_OFFSET; // Pivot offset is 0
  double P4y =  LINK_4_LENGTH * sin(theta5Rad);

  // Vector from P2 to P4
  double P24x = P4x - P2x;
  double P24y = P4y - P2y;
  double P24norm = sqrt(P24x*P24x + P24y*P24y);
  
  // Law of cosines to find the angle at P2 between links a1 and a2
  double cosAngleP2 = (LINK_2_LENGTH*LINK_2_LENGTH + P24norm*P24norm - LINK_3_LENGTH*LINK_3_LENGTH) / 
                     (2.0 * LINK_2_LENGTH * P24norm);
  cosAngleP2 = constrain(cosAngleP2, -1.0, 1.0); // Ensure valid input for acos
  double angleP2 = acos(cosAngleP2);
  
  // Find the angle of the P24 vector
  double angleP24 = atan2(P24y, P24x);
  
  // There are two possible solutions: we can go clockwise or counterclockwise by angleP2
  // Let's try counterclockwise first (add angleP2)
  double angleP3 = angleP24 + angleP2;
  
  // Calculate P3 position
  pos.x = P2x + LINK_2_LENGTH * cos(angleP3);
  pos.y = P2y + LINK_2_LENGTH * sin(angleP3);
  
  return pos;
}

 // TODO REWRITE THIS FUNCTION TO COMPUTE THE CORRECT COORDINATE OUTPUT

// This function writes the current position of the cursor to processing using the 
// Serial communication with the computer.
void sendPosToProcessing(CursorPos pos)
{
  Serial.print("{");
  Serial.print(pos.x, 2);
  Serial.print(",");
  Serial.print(pos.y, 2);
  Serial.println("}");
}

// Reads and processes the forces sent from processing, then echoes them back.
MotorForces readForcesFromProcessing()
{
  static String buf = "";
  MotorForces forces = {0.0, 0.0};

  while (Serial.available())
  {
    char c = Serial.read();
    buf += c;

    if (c == '\n')
    {
      int start = buf.indexOf('<');
      int comma = buf.indexOf(',', start + 1);
      int end   = buf.indexOf('>', comma + 1);

      if (start >= 0 && comma > start && end > comma)
      {
        String s1 = buf.substring(start + 1, comma);
        String s2 = buf.substring(comma + 1, end);
        forces.f1 = s1.toFloat();
        forces.f2 = s2.toFloat();

        #ifdef DEBUG
        // Echo back to Processing
        Serial.print("<");
        Serial.print(forces.f1, 3);
        Serial.print(",");
        Serial.print(forces.f2, 3);
        Serial.println(">");
        #endif
      }

      buf = "";
    }

    if (buf.length() > 50)
      buf = buf.substring(buf.length() - 50);
  }

  return forces;
}

// This function outputs the forces to both motors from the master arduino. It is more
// efficient to pass forces by reference although not strictly necessary. This function runs
// at the end of the void loop and sets the desired force for the motor.
void setMotorForces(const MotorForces &m)
{
  // TODO
}

// This function sets the pulse width modulation frequency of specific
// pins on the Hapkit board. Required to for better operation of sending torques
// to the motor.
void setPwmFrequency(int pin, int divisor)
{
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
  {
    switch (divisor)
    {
      case 1:    mode = 0x01; break;
      case 8:    mode = 0x02; break;
      case 64:   mode = 0x03; break;
      case 256:  mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6)
    {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } 
    else
    {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if (pin == 3 || pin == 11)
  {
    switch (divisor)
    {
      case 1:    mode = 0x01; break;
      case 8:    mode = 0x02; break;
      case 32:   mode = 0x03; break;
      case 64:   mode = 0x04; break;
      case 128:  mode = 0x05; break;
      case 256:  mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

// TODO
void initialiseMotors()
{
  // TODO
}