// *** MASTER ARDUINO CODE, ME327 Group 1, 2025 *** //
// Complete implementation with proper Jacobian and force control
// FIXED VERSION - Corrected I2C speed and force safety + PWM frequency setup

// Make sure debug is off for production!
// #define DEBUG

#include <Encoder.h>
#include <Wire.h>
#include <math.h>

// Communication constants - FIXED I2C SPEED
const uint32_t I2C_BAUD_RATE          = 400000;  // FIXED: Match follower I2C rate
const uint32_t SERIAL_BAUD_RATE       = 115200;  // Keep high for Processing
const uint8_t  MASTER_ADDRESS         = 1;
const uint8_t  LEFT_FOLLOWER_ADDRESS  = 8;
const uint8_t  RIGHT_FOLLOWER_ADDRESS = 9;

// Pin definitions
const int MOTOR_1_PWM_PIN = 5;
const int MOTOR_1_DIR_PIN = 8;
const int MOTOR_5_PWM_PIN = 6;
const int MOTOR_5_DIR_PIN = 7;  

// Hardware parameters
const double COUNTS_PER_REV  = 48.0; 
const double RADIANS_PER_REV = 2.0 * PI;
const double GEAR_RATIO      = 4.4;
const double PULLEY_RADIUS   = 5.1;   // mm
const double SECTOR_RADIUS   = 75.0;  // mm

// Link lengths from your proposal measurements
const double LINK_1_LENGTH   = 127.0; // mm
const double LINK_2_LENGTH   = 152.4; // mm
const double LINK_3_LENGTH   = 152.4; // mm
const double LINK_4_LENGTH   = 127.0; // mm  
const double PIVOT_OFFSET    = 0.0;  // mm

// For backward compatibility with Jacobian code
const double a1 = LINK_1_LENGTH;  // 127.0 mm
const double a2 = LINK_2_LENGTH;  // 152.4 mm
const double a3 = LINK_3_LENGTH;  // 152.4 mm
const double a4 = LINK_4_LENGTH;  // 127.0 mm
const double a5 = PIVOT_OFFSET;   // 10.0 mm

// Motor constant
const double Tstall_max = 51.975245; // N-mm from motor datasheet

const double MECHANICAL_LIMIT_THETA_DEG  = 15.0; // TODO This number needs tuning - it is currently an estimate. but we can fix it will scaling
const double THETA_1_START               = MECHANICAL_LIMIT_THETA_DEG * PI/180.0;
const double THETA_5_START               = (180.0 - MECHANICAL_LIMIT_THETA_DEG) * PI/180.0;

// Global kinematic state
struct KinematicState {
  double theta1, theta5;
  double P2x, P2y, P4x, P4y;
  double P3x, P3y;
  double mag4from2, mag2fromH, mag3fromH;
  // Jacobian elements
  double d1x3, d1y3, d5x3, d5y3;
};

KinematicState currentState;

struct CursorPos {
  double x;
  double y;
};

struct MotorForces {
  float fx;
  float fy;
};

// Function prototypes
int         readFollowerEncoder(int followerAddress, int lastValidCount);
double      countsToMotorRad(int encoderCounts);
void        calculateCompleteKinematics(int countsT1, int countsT5);
void        sendPosToProcessing(CursorPos pos);
MotorForces readForcesFromProcessing();
void        setMotorForces(const MotorForces &forces);
void        applyMotorTorque(int pwmPin, int dirPin, double torque);
void        setPwmFrequency(int pin, int divisor);
void        initialiseMotors();

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(100);
  
  #ifdef DEBUG
  Serial.println("Serial started");
  #endif

  // FIXED: Use correct I2C setup to match followers
  Wire.begin(MASTER_ADDRESS);
  Wire.setClock(I2C_BAUD_RATE);  // Use 400000 to match followers
  
  // Initialize motor pins and PWM frequency
  initialiseMotors();
  
  Serial.println("AstroTouch Pantograph Controller Ready");
}

void loop() {
  static MotorForces motorForces = {0.0, 0.0};
  static int lastValidCountT1 = 0;
  static int lastValidCountT5 = 0;

  // Read encoder values from follower Arduinos
  lastValidCountT1 = -readFollowerEncoder(RIGHT_FOLLOWER_ADDRESS, lastValidCountT1);
  lastValidCountT5 =  readFollowerEncoder(LEFT_FOLLOWER_ADDRESS,  lastValidCountT5);

  #ifdef DEBUG
  Serial.print("Raw T1: "); Serial.println(lastValidCountT1);
  Serial.print("Raw T5: "); Serial.println(lastValidCountT5);
  #endif

  // Calculate complete kinematics using Jacobian
  calculateCompleteKinematics(lastValidCountT1, lastValidCountT5);
  
  // Send position to Processing with workspace offset
  CursorPos pos = {currentState.P3x + 200, currentState.P3y};
  sendPosToProcessing(pos);

  // Read forces from Processing and apply to motors
  motorForces = readForcesFromProcessing();
  setMotorForces(motorForces);
  
  delay(1); // 1kHz control loop
}

int readFollowerEncoder(int followerAddress, int lastValidCount) {
  if (Wire.requestFrom(followerAddress, 3) == 3) {
    uint8_t srcID = Wire.read();
    uint8_t loByte = Wire.read();
    uint8_t hiByte = Wire.read();

    #ifdef DEBUG
    Serial.print("Received from "); Serial.print(followerAddress);
    Serial.print(": 0x"); Serial.print(srcID, HEX);
    Serial.print(" 0x"); Serial.print(loByte, HEX);
    Serial.print(" 0x"); Serial.println(hiByte, HEX);
    #endif

    lastValidCount = (int16_t)((hiByte << 8) | loByte);
    return lastValidCount;
  } else {
    #ifdef DEBUG
    Serial.print("I2C read failed for address "); Serial.println(followerAddress);
    #endif
    return lastValidCount; // Return last known valid count
  }
}

double countsToMotorRad(int encoderCounts) {
  return (encoderCounts / COUNTS_PER_REV / GEAR_RATIO) * RADIANS_PER_REV;
}

void calculateCompleteKinematics(int countsT1, int countsT5) {
  // Convert encoder counts to motor angles
  double motorTheta1 = countsToMotorRad(countsT1);
  double motorTheta5 = countsToMotorRad(countsT5);
  
  // Get joint angles
  currentState.theta1 = THETA_1_START - (PULLEY_RADIUS/SECTOR_RADIUS) * motorTheta1;
  currentState.theta5 = THETA_5_START - (PULLEY_RADIUS/SECTOR_RADIUS) * motorTheta5;
  
  // Calculate link positions based on pantograph kinematics
  currentState.P2x = -LINK_1_LENGTH * cos(currentState.theta1);
  currentState.P2y =  LINK_1_LENGTH * sin(currentState.theta1);
  currentState.P4x = -LINK_4_LENGTH * cos(currentState.theta5) + PIVOT_OFFSET;
  currentState.P4y =  LINK_4_LENGTH * sin(currentState.theta5);

  // Vector from P2 to P4
  double P24x = currentState.P4x - currentState.P2x;
  double P24y = currentState.P4y - currentState.P2y;
  currentState.mag4from2 = sqrt(P24x*P24x + P24y*P24y);
  
  // Law of cosines to find the angle at P2 between links a1 and a2
  double cosAngleP2 = (LINK_2_LENGTH*LINK_2_LENGTH + currentState.mag4from2*currentState.mag4from2 - LINK_3_LENGTH*LINK_3_LENGTH) / 
                     (2.0 * LINK_2_LENGTH * currentState.mag4from2);
  cosAngleP2 = constrain(cosAngleP2, -1.0, 1.0);
  double angleP2 = acos(cosAngleP2);
  
  // Find the angle of the P24 vector
  double angleP24 = atan2(P24y, P24x);
  
  // Calculate P3 position (end effector)
  double angleP3 = angleP24 + angleP2;
  currentState.P3x = currentState.P2x + LINK_2_LENGTH * cos(angleP3);
  currentState.P3y = currentState.P2y + LINK_2_LENGTH * sin(angleP3);
  
  // Calculate parameters for Jacobian
  currentState.mag2fromH = ((a2*a2) - (a3*a3) + (currentState.mag4from2 * currentState.mag4from2)) / 
                          (2.0 * currentState.mag4from2);
  currentState.mag3fromH = sqrt((a2*a2) - (currentState.mag2fromH * currentState.mag2fromH));
  
  // COMPLETE JACOBIAN CALCULATION (from Campion paper)
  double d = currentState.mag4from2;
  double b = currentState.mag2fromH;
  double h = currentState.mag3fromH;
  
  // Joint coordinate derivatives
  double d1x2 = -a1 * sin(currentState.theta1);
  double d1y2 =  a1 * cos(currentState.theta1);
  double d5x4 = -a4 * sin(currentState.theta5);
  double d5y4 =  a4 * cos(currentState.theta5);
  double d1y4 = 0, d1x4 = 0, d5y2 = 0, d5x2 = 0;
  
  // Magnitude derivatives
  double d1d = ((currentState.P4x - currentState.P2x) * (d1x4 - d1x2) + 
                (currentState.P4y - currentState.P2y) * (d1y4 - d1y2)) / d;
  double d5d = ((currentState.P4x - currentState.P2x) * (d5x4 - d5x2) + 
                (currentState.P4y - currentState.P2y) * (d5y4 - d5y2)) / d;
  
  double d1b = d1d - d1d * ((a2*a2) - (a3*a3) + (d*d)) / (2*d*d);
  double d5b = d5d - d5d * ((a2*a2) - (a3*a3) + (d*d)) / (2*d*d);
  
  double d1h = (-b * d1b) / h;
  double d5h = (-b * d5b) / h;
  
  // Point h coordinate derivatives
  double d1yh = d1y2 + (d1b*d - d1d*b) * (currentState.P4y - currentState.P2y) / (d*d) + 
                (b/d) * (d1y4 - d1y2);
  double d5yh = d5y2 + (d5b*d - d5d*b) * (currentState.P4y - currentState.P2y) / (d*d) + 
                (b/d) * (d5y4 - d5y2);
  double d1xh = d1x2 + (d1b*d - d1d*b) * (currentState.P4x - currentState.P2x) / (d*d) + 
                (b/d) * (d1x4 - d1x2);
  double d5xh = d5x2 + (d5b*d - d5d*b) * (currentState.P4x - currentState.P2x) / (d*d) + 
                (b/d) * (d5x4 - d5x2);
  
  // Final Jacobian entries
  currentState.d1y3 = d1yh - (h/d) * (d1x4 - d1x2) - 
                      (d1h*d - d1d*h) / (d*d) * (currentState.P4x - currentState.P2x);
  currentState.d5y3 = d5yh - (h/d) * (d5x4 - d5x2) - 
                      (d5h*d - d5d*h) / (d*d) * (currentState.P4x - currentState.P2x);
  currentState.d1x3 = d1xh + (h/d) * (d1y4 - d1y2) + 
                      (d1h*d - d1d*h) / (d*d) * (currentState.P4y - currentState.P2y);
  currentState.d5x3 = d5xh + (h/d) * (d5y4 - d5y2) + 
                      (d5h*d - d5d*h) / (d*d) * (currentState.P4y - currentState.P2y);
}

void sendPosToProcessing(CursorPos pos) {
  // Send space-separated format that Processing expects
  Serial.print(pos.x, 3);
  Serial.print(" ");
  Serial.println(pos.y, 3);
}

MotorForces readForcesFromProcessing() {
  static String buf = "";
  static MotorForces smoothedForces = {0.0, 0.0};
  static MotorForces lastForces = {0.0, 0.0}; // ADDED: For rate limiting
  MotorForces newForces = {0.0, 0.0};

  while (Serial.available()) {
    char c = Serial.read();
    buf += c;

    if (c == '\n') {
      // Parse "fx fy" format from Processing
      int spaceIndex = buf.indexOf(' ');
      if (spaceIndex > 0) {
        String fxStr = buf.substring(0, spaceIndex);
        String fyStr = buf.substring(spaceIndex + 1);
        newForces.fx = fxStr.toFloat();
        newForces.fy = fyStr.toFloat();
        
        // ENHANCED: Safety limits with rate limiting
        newForces.fx = constrain(newForces.fx, -12.0, 12.0); // Reduced max force
        newForces.fy = constrain(newForces.fy, -12.0, 12.0);
        
        // ADDED: Rate limiting to prevent sudden force jumps
        float maxForceChange = 2.0; // N per cycle
        float fxChange = constrain(newForces.fx - lastForces.fx, -maxForceChange, maxForceChange);
        float fyChange = constrain(newForces.fy - lastForces.fy, -maxForceChange, maxForceChange);
        
        newForces.fx = lastForces.fx + fxChange;
        newForces.fy = lastForces.fy + fyChange;
        lastForces = newForces;
        
        // Low-pass filter for smooth force application
        float alpha = 0.7; // Slightly more filtering
        smoothedForces.fx = alpha * smoothedForces.fx + (1-alpha) * newForces.fx;
        smoothedForces.fy = alpha * smoothedForces.fy + (1-alpha) * newForces.fy;
      }
      buf = "";
    }

    if (buf.length() > 50)
      buf = buf.substring(buf.length() - 50);
  }

  return smoothedForces;
}

void setMotorForces(const MotorForces &forces) {
  // Convert forces [N] to joint torques [N-mm] using Jacobian transpose
  // [T1; T5] = [d1x3, d1y3; d5x3, d5y3]^T * [Fx; Fy]
  double T1 = currentState.d1x3 * forces.fx + currentState.d1y3 * forces.fy;
  double T5 = currentState.d5x3 * forces.fx + currentState.d5y3 * forces.fy;
  
  // Convert joint torques to motor torques via capstan ratio
  double Tmotor1 = (PULLEY_RADIUS/SECTOR_RADIUS) * T1;
  double Tmotor5 = (PULLEY_RADIUS/SECTOR_RADIUS) * T5;
  
  // ADDED: Additional torque safety limiting
  Tmotor1 = constrain(Tmotor1, -Tstall_max * 0.8, Tstall_max * 0.8);
  Tmotor5 = constrain(Tmotor5, -Tstall_max * 0.8, Tstall_max * 0.8);
  
  #ifdef DEBUG
  Serial.print("Forces: Fx="); Serial.print(forces.fx);
  Serial.print(" Fy="); Serial.print(forces.fy);
  Serial.print(" -> T1="); Serial.print(Tmotor1);
  Serial.print(" T5="); Serial.println(Tmotor5);
  #endif
  
  // Apply motor torques
  applyMotorTorque(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN, Tmotor1);
  applyMotorTorque(MOTOR_5_PWM_PIN, MOTOR_5_DIR_PIN, Tmotor5);
}

void applyMotorTorque(int pwmPin, int dirPin, double torque) {
  // Set direction
  digitalWrite(dirPin, torque > 0 ? HIGH : LOW);
  
  // Calculate PWM duty cycle
  float duty = abs(torque) / Tstall_max;
  duty = constrain(duty, 0, 0.9); // Limit to 90% for safety
  int pwmValue = (int)(duty * 255);
  analogWrite(pwmPin, pwmValue);
}

// This function sets the pulse width modulation frequency of specific
// pins on the Hapkit board. Required for better operation of sending torques
// to the motor.
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1:    mode = 0x01; break;
      case 8:    mode = 0x02; break;
      case 64:   mode = 0x03; break;
      case 256:  mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } 
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1:    mode = 0x01; break;
      case 8:    mode = 0x02; break;
      case 32:   mode = 0x03; break;
      case 64:   mode = 0x04; break;
      case 128:  mode = 0x05; break;
      case 256:  mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

// Initialize motors with proper PWM frequency and pin setup
void initialiseMotors() {
  // Initialize motor pins
  pinMode(MOTOR_1_PWM_PIN, OUTPUT);
  pinMode(MOTOR_1_DIR_PIN, OUTPUT);
  pinMode(MOTOR_5_PWM_PIN, OUTPUT);  
  pinMode(MOTOR_5_DIR_PIN, OUTPUT);
  
  // Set motors to zero initially
  analogWrite(MOTOR_1_PWM_PIN, 0);
  analogWrite(MOTOR_5_PWM_PIN, 0);
  digitalWrite(MOTOR_1_DIR_PIN, LOW);
  digitalWrite(MOTOR_5_DIR_PIN, LOW);
  
  // Set PWM frequency for better motor control
  // Higher frequency reduces motor noise and improves force rendering
  // Using divisor of 8 gives ~3.9kHz PWM frequency on pins 5&6
  setPwmFrequency(MOTOR_1_PWM_PIN, 8);  // Pin 5
  setPwmFrequency(MOTOR_5_PWM_PIN, 8);  // Pin 6
  
  #ifdef DEBUG
  Serial.println("Motors initialized with optimized PWM frequency");
  #endif
}