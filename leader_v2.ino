// *** MASTER ARDUINO CODE, ME327 Group 1, 2025 *** //
// TODO  

// Make sure debug is off otherwise it becomes extremely slow due to the processes that
// are being run!!
// #define DEBUG

#include <Encoder.h>
#include <SoftwareSerial.h>

// Define communication constants.
static const uint32_t BAUD_RATE  = 9600;
static const uint8_t  START_BYTE = 0xAA;  // packet start marker
static const uint8_t  FOLL_ID    = 0x01;  // follower ID tag

// Define pins
const int ENCODER_PIN_1      = 2;
const int ENCODER_PIN_2      = 3;
const int MOTOR_1_PWM_PIN    = 5;
const int MOTOR_1_DIR_PIN    = 8;
const int MOTOR_5_PWM_PIN    = 6;
const int MOTOR_5_DIR_PIN    = 7;
const int SOFTWARE_SERIAL_RX = 10;
const int SOFTWARE_SERIAL_TX = 11;

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

// First encoder, for theta1
Encoder encT1(ENCODER_PIN_1, ENCODER_PIN_2);

// Create a software serial port using the software serial library. This
// allows the Arduino Uno to act as if it had multiple serial ports. This
// allows us to communicate with both the follower Arduino and processing
// simultaneously.
SoftwareSerial Serial1(SOFTWARE_SERIAL_RX, SOFTWARE_SERIAL_TX);

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
int            readFollowerEncoder();
CursorPos      calculateXYPosition(int countsT1, int countsT5);
void           sendPosToProcessing(CursorPos pos);
MotorForces    readForcesFromProcessing();
void           setMotorForces(const MotorForces &forces);

// This function runs once on startup.
void setup()
{
  // Configure Encoder pins
  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);

  // Set encoder to 0 on power on.
  encT1.write(0);
  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);
}

// This function runs continuously.
void loop()
{
  static MotorForces motorForces = { 0.0, 0.0 };

  // Read the values of both encoders.
  long countsT1 = encT1.read();
  long countsT5 = readFollowerEncoder();

  CursorPos pos = calculateXYPosition(countsT1, countsT5);
  sendPosToProcessing(pos);

  motorForces = readForcesFromProcessing();
  setMotorForces(motorForces);
}

// This function reads the information sent from the follower Arduino and
// returns the decoded encoder count as an integer. It is a complex function
// but for any future groups looking modify this system, you should not need 
// to edit this function if you keep the follower code exactly the same. Bear
// in mind, this function understands only the protocol defined in the follower 
// code, i.e. start byte = 0xAA, follower source ID = 0x01 etc.
int readFollowerEncoder()
{
  #ifdef DEBUG
  Serial.println("entered read follower");
  #endif

  static RxState state         = WAIT_START;
  static uint8_t  srcByte      = 0;
  static uint8_t  lowByte      = 0;
  static uint8_t  highByte     = 0;
  static uint8_t  checksumByte = 0;
  static int      lastCount    = 0;
  int bytesProcessed = 0;
  const int maxBytesToProcess = 20;

  Serial1.listen();
  while (Serial1.available() && bytesProcessed < maxBytesToProcess)
  {
    bytesProcessed++;
    uint8_t b = Serial1.read();

    #ifdef DEBUG
    Serial.print("0x");
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');
    #endif

    switch (state)
    {
      case WAIT_START:
        if (b == START_BYTE) state = READ_SRC;
        break;
      case READ_SRC:
        srcByte = b;
        state   = READ_LOW;
        break;
      case READ_LOW:
        lowByte = b;
        state   = READ_HIGH;
        break;
      case READ_HIGH:
        highByte = b;
        state    = READ_CHECK;
        break;
      case READ_CHECK:
        checksumByte = b;
        // verify checksum and source ID
        if (checksumByte == (srcByte ^ lowByte ^ highByte)
            && srcByte == FOLL_ID)
        {
          // valid packet
          lastCount = (int16_t)((highByte << 8) | lowByte);

          #ifdef DEBUG
          Serial.print("Valid packet received. Count: ");
          Serial.println(lastCount);
          #endif
        }
        else
        {
          #ifdef DEBUG
          Serial.println("Invalid checksum or source ID");
          #endif
        }
        state = WAIT_START;
        break;
    }
  }

  #ifdef DEBUG
  if (bytesProcessed >= maxBytesToProcess)
    Serial.println("Hit maximum bytes per call limit");
  #endif

  return lastCount;
}

// This function calculates and returns the current position of the cursor based
// on the current encoder counts.
CursorPos calculateXYPosition(int countsT1, int countsT5)
{
  CursorPos pos;
  pos.x = (double)countsT1;
  pos.y = (double)countsT5;
  return pos;
} // TODO REWRITE THIS FUNCTION TO COMPUTE THE CORRECT COORDINATE OUTPUT

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
  MotorForces forces = { 0.0, 0.0 };

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
// pins on the Hapkit board. Required to for better operation of sending forces to the 
// motor.
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