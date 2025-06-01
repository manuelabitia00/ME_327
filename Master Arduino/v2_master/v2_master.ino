// *** MASTER ARDUINO CODE, ME327 Group 1, 2025 *** //
// TODO Throughout this code, left encoder is referring to the encoder that is on 
// the left when looking at the back face of the pantograph.  

// Make sure debug is off otherwise it becomes extremely slow due to the processes that
// are being run!!
// #define DEBUG

#include <Encoder.h>
#include <SoftwareSerial.h>
#include <math.h>

// Define communication constants.
static const uint32_t BAUD_RATE        = 9600;
static const uint8_t  START_BYTE       = 0xAA;  // packet start marker
static const uint8_t  LEFT_ARDUINO_ID  = 0x01;  // The id tag to identify comms from itsy bitsy reading the left encoder
static const uint8_t  RIGHT_ARDUINO_ID = 0x02;  // The id tag we identify comms from itsy bitsy reading the right encoder

// Define pins
// const int ENCODER_PIN_1        = 2;
// const int ENCODER_PIN_2        = 3;
const int MOTOR_1_PWM_PIN      = 5;
const int MOTOR_1_DIR_PIN      = 8;
const int MOTOR_5_PWM_PIN      = 6;
const int MOTOR_5_DIR_PIN      = 7;
const int SOFTWARE_SERIAL_RX_1 = 2;  // corresponds to the left arduino motor
const int SOFTWARE_SERIAL_RX_2 = 3;  // corresponds to the right arduino motor
const int SOFTWARE_SERIAL_TX_1 = -1; // not used, only receiving 
const int SOFTWARE_SERIAL_TX_2 = -1; // not used, only receiving

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
const double PIVOT_OFFSET    = 10.0;  // TODO This is a5 in the diagram but equals 0

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

// Create a software serial port using the software serial library. This
// allows the Arduino Uno to act as if it had multiple serial ports. This
// allows us to communicate with both follower Arduinos and processing
// simultaneously. Serial 1 is reading left enc, serial 2 is readuing right enc.
SoftwareSerial Serial1(SOFTWARE_SERIAL_RX_1, SOFTWARE_SERIAL_TX_1);
SoftwareSerial Serial2(SOFTWARE_SERIAL_RX_2, SOFTWARE_SERIAL_TX_2);

// Parser states for incoming packets from the follower arduino
enum RxState
{
  WAIT_START,
  READ_SRC,
  READ_LOW,
  READ_HIGH,
  READ_CHECK
};

// Declare function prototypes
int            readFollowerEncoder(const int idTag);
CursorPos      calculateXYPosition(int countsT1, int countsT5);
void           sendPosToProcessing(CursorPos pos);
MotorForces    readForcesFromProcessing();
void           setMotorForces(const MotorForces &forces);

// This function runs once on startup.
void setup()
{
  // // Configure Encoder pins
  // pinMode(ENCODER_PIN_1, INPUT);
  // pinMode(ENCODER_PIN_2, INPUT);

  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);
  Serial2.begin(BAUD_RATE);
}

// This function runs continuously.
void loop()
{
  static MotorForces motorForces = {0.0, 0.0};
  static bool readLeftThisLoop = true;
  static long countsT1 = 0;
  static long countsT5 = 0;
  
  // Read only LEFT encoder this loop
  if (readLeftThisLoop)
  {
    
    Serial1.listen();
    delay(10);  // Give time to stabilize
    countsT5 = readFollowerEncoder(LEFT_ARDUINO_ID);
  }
  // Read only RIGHT encoder this loop
  else 
  {
    Serial2.listen();
    delay(10);  // Give time to stabilize
    countsT1 = -readFollowerEncoder(RIGHT_ARDUINO_ID);
  }
  
  // Toggle for next loop
  readLeftThisLoop = !readLeftThisLoop;
  
  // Always use the latest valid values for both encoders
  // Serial.print("countT1: ");
  // Serial.print(countsT1);
  // Serial.print(", countT5: ");
  // Serial.println(countsT5);
  
  CursorPos pos = calculateXYPosition(countsT1, countsT5);
  pos.x += 200;
  sendPosToProcessing(pos);

  // motorForces = readForcesFromProcessing();
  // setMotorForces(motorForces);
}

// This function reads the information sent from the follower Arduino and
// returns the decoded encoder count as an integer. It is a complex function
// but for any future groups looking modify this system, you should not need 
// to edit this function if you keep the follower code exactly the same. Bear
// in mind, this function understands only the protocol defined in the follower 
// code, i.e. start byte = 0xAA, follower source ID = 0x01 etc.
int readFollowerEncoder(const int idTag)
{
  #ifdef DEBUG
  Serial.println("ENTERED: readFollowerEncoder");
  Serial.print("Reading ");
  Serial.print(idTag == LEFT_ARDUINO_ID ? "LEFT" : "RIGHT");
  Serial.println(" encoder");
  #endif

  // Create the byte architecture for reading bytes from the left encoder
  static RxState stateLeft         = WAIT_START;
  static uint8_t  srcByteLeft      = 0;
  static uint8_t  loByteLeft       = 0;
  static uint8_t  hiByteLeft       = 0;
  static uint8_t  checksumByteLeft = 0;
  static int      lastCountLeft    = 0;

  // Create the byte architecture for reading bytes from the right encoder
  static RxState stateRight         = WAIT_START;
  static uint8_t  srcByteRight      = 0;
  static uint8_t  loByteRight       = 0;
  static uint8_t  hiByteRight       = 0;
  static uint8_t  checksumByteRight = 0;
  static int      lastCountRight    = 0;

  int bytesProcessed = 0;
  const int maxBytesToProcess = 40;

  // Create pointer variables that allow us to set the serial we are going to read
  // from on different function calls.
  RxState* state;
  uint8_t* srcByte;
  uint8_t* loByte;
  uint8_t* hiByte;
  uint8_t* checksumByte;
  int*     lastCount;
  SoftwareSerial* activeSerial;

  // Based on the input idTag, set the pointers to point to the addresses of the 
  // variables we are going to modify 
  if (idTag == LEFT_ARDUINO_ID)
  {
    state        = &stateLeft;
    srcByte      = &srcByteLeft;
    loByte       = &loByteLeft;
    hiByte       = &hiByteLeft;
    checksumByte = &checksumByteLeft;
    lastCount    = &lastCountLeft;
    activeSerial = &Serial1;
  }
  else if (idTag == RIGHT_ARDUINO_ID)
  {    
    state        = &stateRight;
    srcByte      = &srcByteRight;
    loByte       = &loByteRight;
    hiByte       = &hiByteRight;
    checksumByte = &checksumByteRight;
    lastCount    = &lastCountRight;
    activeSerial = &Serial2;
  }
  else
  {
    Serial.print("readFollowerEncoder: UNKNOWN ID TAG SPECIFIED");
    Serial.println(idTag);
    return 0;
  }

  while (activeSerial->available() && bytesProcessed < maxBytesToProcess)
  {
    #ifdef DEBUG
    Serial.println("entered while loop");
    #endif

    bytesProcessed++;
    uint8_t b = activeSerial->read();

    #ifdef DEBUG
    Serial.print("0x");
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');
    #endif

    switch (*state)
    {
      case WAIT_START:
        if (b == START_BYTE) *state = READ_SRC;
        break;
      case READ_SRC:
        *srcByte = b;
        *state   = READ_LOW;
        break;
      case READ_LOW:
        *loByte = b;
        *state  = READ_HIGH;
        break;
      case READ_HIGH:
        *hiByte = b;
        *state  = READ_CHECK;
        break;
      case READ_CHECK:
        *checksumByte = b;
        // verify checksum and source ID
        if (*checksumByte == (*srcByte ^ *loByte ^ *hiByte)
            && *srcByte == idTag)
        {
          // valid packet
          *lastCount = (int16_t)((*hiByte << 8) | *loByte);

          #ifdef DEBUG
          Serial.print("Valid packet received. Count: ");
          Serial.println(*lastCount);
          #endif
        }
        else
        {
          #ifdef DEBUG
          Serial.println("Invalid checksum or source ID");
          #endif
        }
        *state = WAIT_START;
        break;
    }
  }

  #ifdef DEBUG
  if (bytesProcessed >= maxBytesToProcess)
    Serial.println("Hit maximum bytes per call limit");
  #endif

  return *lastCount;
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