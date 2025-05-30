#include <Encoder.h>

/* ENCODERS */
#define COUNTS_PER_REV 48.0
#define RADS_PER_REV (2*3.14159265355)
#define PI 3.1415926535
#define GEAR_RATIO 4.4 // For computing angle 
Encoder encT1(2,3); // First encoder, for theta1
Encoder encT5(4,5); // Second encoder, for theta5 (adjust pins as needed)
/* ENCODERS (End) */

/* MOTORS */
// Pins
const int MOTOR_1_PWM_PIN = 6; // PWM output pin for motor 1
const int MOTOR_1_DIR_PIN = 7; // direction output pin for motor 1
const int MOTOR_5_PWM_PIN = 9; // PWM output pin for motor 5
const int MOTOR_5_DIR_PIN = 8; // direction output pin for motor 5

// For setting PWM frequency
const int PWM_DIVIDER = 1;
const int PRESCALE_FACTOR = 64 / PWM_DIVIDER;

// For computing torques
static const double Tstall_max = 51.975245; // [N-mm]. From motor data sheet.

// For mapping to motor angles
static int counts1, counts5;
static double motorAngle1, motorAngle5; // [rad]. For use in computation
static double motorAngle1_deg, motorAngle5_deg; // [º]. For printing only

/* MOTORS (End) */

/* USER POSITION */
// For mapping motor angles to Theta1 and Theta5
static const double r_pulley = 5.1, r_sector = 75.0; // [mm]. Capstan drive radii

#define STARTER_ANGLE 14 // Modify for tuning/calibration
#define THETA_1_START (STARTER_ANGLE*PI/180) // Starting angle for motor 1
#define THETA_5_START ((180-STARTER_ANGLE)*PI/180) // Starting angle for motor 5
static double theta1, theta5;

// Link lengths (per Campion paper and your CAD)
static const double a1 = 63.0; // [mm]
static const double a2 = 75.0; // [mm]
static const double a3 = 75.0; // [mm]. Same as a2.
static const double a4 = 63.0; // [mm]. Same as a1.
static const double a5 = 25.0; // [mm]

// Pantograph kinematics
static double P2x, P2y, P4x, P4y, mag4from2, mag2fromH, Phx, Phy, mag3fromH, P3x, P3y;

// User speed calculation
static double P3x_prev, P3y_prev, dP3x, dP3y, dP3x_filt, dP3y_filt;

/* USER POSITION (End) */

/* FORCES AND TORQUES */ 
// Jacobian matrix elements
static double d, b, h; // Magnitude variables
static double d1x2, d1y2, d5x4, d5y4, d1y4, d1x4, d5y2, d5x2; // Joint coordinate derivatives
static double d1d, d5d, d1b, d5b, d1h, d5h; // Magnitude variable derivatives
static double d1yh, d1xh, d5yh, d5xh; // Point h coordinate derivatives
static double d1y3, d5y3, d1x3, d5x3; // User coordinate derivatives

// Force and torque variables
static double Fx, Fy; // Forces from Processing [N]
static double T1, T5, Tmotor1, Tmotor5; // Torques [N-mm]

/* FORCES AND TORQUES (End) */ 

/* SERIAL COMMUNICATION */
String inputString = "";
boolean stringComplete = false;
/* SERIAL COMMUNICATION (End) */

void setup() {
  Serial.begin(115200);
  
  // Initialize motors
  initMotors();
  
  // Configure Encoder pins
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  
  // Initialize encoder counts
  encT1.write(0);
  encT5.write(0);
  
  // Initialize previous variables
  P3x_prev = 0;
  P3y_prev = 0;
  dP3x = 0;
  dP3y = 0;
  dP3x_filt = 0;
  dP3y_filt = 0;
  
  // Initialize forces
  Fx = 0;
  Fy = 0;
  
  // Reserve space for input string
  inputString.reserve(200);
  
  Serial.println("Arduino Pantograph Haptic Controller Ready");
}

void loop() {
  // Read encoder positions and compute kinematics
  getMotorAngles();
  getUserPos();
  getUserSpeed();
  
  // Read force commands from Processing
  if (stringComplete) {
    parseForceCommand();
    stringComplete = false;
    inputString = "";
  }
  
  // Compute Jacobian for force-to-torque mapping
  getJacobian();
  
  // Calculate motor torques from desired forces
  updateDutyCycles();
  
  // Send position data to Processing
  sendPositionToProcessing();
  
  // Reset previous values for next iteration
  resetPrevVals();
  
  delay(1); // Small delay for stability (1kHz control loop)
}

/****************************************************************************
 Function: getMotorAngles
 Description: Get angles of motor shafts from encoders
****************************************************************************/
void getMotorAngles(void) {
  /* ENCODER 1 */
  counts1 = encT1.read(); // Read encoder counts
  motorAngle1 = (counts1/COUNTS_PER_REV/GEAR_RATIO)*RADS_PER_REV; // [rads]
  motorAngle1_deg = motorAngle1 * 180/PI;

  /* ENCODER 5 */
  counts5 = encT5.read(); // Read encoder counts
  motorAngle5 = (counts5/COUNTS_PER_REV/GEAR_RATIO)*RADS_PER_REV; // [rads]
  motorAngle5_deg = motorAngle5 * 180/PI;
}

/****************************************************************************
 Function: getUserPos
 Description: Get user positions using pantograph kinematics
****************************************************************************/
void getUserPos(void) {
  // Get Theta1 and Theta5 [rads] from motor angles
  theta1 = THETA_1_START - ((r_pulley/r_sector)*motorAngle1); 
  theta5 = THETA_5_START - ((r_pulley/r_sector)*motorAngle5);

  // Get P2 and P4 [mm] - joint positions
  P2x = a1*cos(theta1);
  P2y = a1*sin(theta1);
  P4x = a4*cos(theta5) - a5;
  P4y = a4*sin(theta5);

  // Get magnitudes [mm]
  mag4from2 = sqrt((P4x-P2x)*(P4x-P2x) + (P4y-P2y)*(P4y-P2y));
  mag2fromH = ((a2*a2) - (a3*a3) + (mag4from2*mag4from2))/(2*mag4from2);

  // Get Ph [mm] - intersection point
  Phx = P2x + (mag2fromH/mag4from2)*(P4x-P2x);
  Phy = P2y + (mag2fromH/mag4from2)*(P4y-P2y);

  // Get P3x and P3y [mm] - end effector position
  mag3fromH = sqrt((a2*a2) - (mag2fromH*mag2fromH));
  P3x = Phx + (mag3fromH/mag4from2)*(P4y-P2y);
  P3y = Phy - (mag3fromH/mag4from2)*(P4x-P2x);
}

/****************************************************************************
 Function: getUserSpeed
 Description: Get user x and y velocities with filtered time-based derivative
****************************************************************************/
void getUserSpeed(void) {
  // Calculate velocity with loop time estimation (assuming 1kHz)
  dP3x = (P3x - P3x_prev) / 0.001; // [mm/s]
  dP3y = (P3y - P3y_prev) / 0.001; // [mm/s]

  // Apply simple low-pass filter
  dP3x_filt = 0.9*dP3x_filt + 0.1*dP3x;
  dP3y_filt = 0.9*dP3y_filt + 0.1*dP3y;
}

/****************************************************************************
 Function: getJacobian
 Description: Get Jacobian matrix for force-to-torque transformation
****************************************************************************/
void getJacobian(void) {
  // Update magnitude variables
  d = mag4from2;
  b = mag2fromH;
  h = mag3fromH;

  // Joint coordinate derivatives [mm]
  d1x2 = -a1*sin(theta1);
  d1y2 = a1*cos(theta1);
  d5x4 = -a4*sin(theta5);
  d5y4 = a4*cos(theta5);
  d1y4 = d1x4 = d5y2 = d5x2 = 0.0;

  // Magnitude derivatives
  d1d = ((P4x-P2x)*(d1x4-d1x2) + (P4y-P2y)*(d1y4-d1y2))/d;
  d5d = ((P4x-P2x)*(d5x4-d5x2) + (P4y-P2y)*(d5y4-d5y2))/d;

  d1b = d1d - d1d*((a2*a2) - (a3*a3) + (d*d))/(2*d*d);
  d5b = d5d - d5d*((a2*a2) - (a3*a3) + (d*d))/(2*d*d);

  d1h = (-b*d1b)/h;
  d5h = (-b*d5b)/h;

  // Point h coordinate derivatives
  d1yh = d1y2 + (d1b*d - d1d*b)*(P4y - P2y)/(d*d) + (b/d)*(d1y4 - d1y2);
  d5yh = d5y2 + (d5b*d - d5d*b)*(P4y - P2y)/(d*d) + (b/d)*(d5y4 - d5y2);
  d1xh = d1x2 + (d1b*d - d1d*b)*(P4x - P2x)/(d*d) + (b/d)*(d1x4 - d1x2);
  d5xh = d5x2 + (d5b*d - d5d*b)*(P4x - P2x)/(d*d) + (b/d)*(d5x4 - d5x2);

  // Jacobian entries [mm]
  d1y3 = d1yh - (h/d)*(d1x4-d1x2) - (d1h*d - d1d*h)/(d*d)*(P4x - P2x);
  d5y3 = d5yh - (h/d)*(d5x4-d5x2) - (d5h*d - d5d*h)/(d*d)*(P4x - P2x);
  d1x3 = d1xh + (h/d)*(d1y4-d1y2) + (d1h*d - d1d*h)/(d*d)*(P4y - P2y);
  d5x3 = d5xh + (h/d)*(d5y4-d5y2) + (d5h*d - d5d*h)/(d*d)*(P4y - P2y);
}

/****************************************************************************
 Function: updateDutyCycles
 Description: Calculate motor torques from forces and update PWM
****************************************************************************/
void updateDutyCycles(void) {
  // Convert forces [N] to joint torques [N-mm] using Jacobian transpose
  // [T1; T5] = [d1x3, d1y3; d5x3, d5y3] * [Fx; Fy]
  T1 = d1x3*Fx + d1y3*Fy; // [N-mm]
  T5 = d5x3*Fx + d5y3*Fy; // [N-mm]
  
  // Map joint torques to motor torques via capstan ratio
  Tmotor1 = (r_pulley/r_sector)*T1; // [N-mm]
  Tmotor5 = (r_pulley/r_sector)*T5; // [N-mm]

  // MOTOR 1
  float duty1;
  unsigned int output1;
  
  // Set direction
  if(Tmotor1 > 0) { 
    digitalWrite(MOTOR_1_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_1_DIR_PIN, LOW);
  }
  
  // Calculate duty cycle (linear relationship with torque)
  duty1 = abs(Tmotor1)/Tstall_max;
  duty1 = constrain(duty1, 0, 1);
  output1 = (int)(duty1 * 255);
  analogWrite(MOTOR_1_PWM_PIN, output1);

  // MOTOR 5
  float duty5;
  unsigned int output5;
  
  // Set direction
  if(Tmotor5 > 0) { 
    digitalWrite(MOTOR_5_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_5_DIR_PIN, LOW);
  }
  
  // Calculate duty cycle
  duty5 = abs(Tmotor5)/Tstall_max;
  duty5 = constrain(duty5, 0, 1);
  output5 = (int)(duty5 * 255);
  analogWrite(MOTOR_5_PWM_PIN, output5);
}

/****************************************************************************
 Function: parseForceCommand
 Description: Parse force commands from Processing ("Fx Fy")
****************************************************************************/
void parseForceCommand(void) {
  // Parse the input string for force values
  int spaceIndex = inputString.indexOf(' ');
  if (spaceIndex > 0) {
    String fxStr = inputString.substring(0, spaceIndex);
    String fyStr = inputString.substring(spaceIndex + 1);
    
    Fx = fxStr.toFloat(); // [N]
    Fy = fyStr.toFloat(); // [N]
    
    // Optional: Add safety limits
    Fx = constrain(Fx, -15.0, 15.0);
    Fy = constrain(Fy, -15.0, 15.0);
  }
}

/****************************************************************************
 Function: sendPositionToProcessing
 Description: Send current pantograph position to Processing
****************************************************************************/
void sendPositionToProcessing(void) {
  // Send position data in format "x y" where x,y are in mm
  Serial.print(P3x, 3);
  Serial.print(" ");
  Serial.println(P3y, 3);
}

/****************************************************************************
 Function: resetPrevVals
 Description: Reset previous values for next loop iteration
****************************************************************************/
void resetPrevVals(void) {
  P3x_prev = P3x;
  P3y_prev = P3y;
}

/****************************************************************************
 Function: initMotors
 Description: Initialize both motors and PWM settings
****************************************************************************/
void initMotors() {
  // Set PWM frequency for both motors
  set_pwm_frequency(MOTOR_1_PWM_PIN, PWM_DIVIDER);
  set_pwm_frequency(MOTOR_5_PWM_PIN, PWM_DIVIDER);
  
  // Configure motor 1 pins
  pinMode(MOTOR_1_PWM_PIN, OUTPUT);
  pinMode(MOTOR_1_DIR_PIN, OUTPUT);
  analogWrite(MOTOR_1_PWM_PIN, 0);
  digitalWrite(MOTOR_1_DIR_PIN, LOW);
  
  // Configure motor 5 pins  
  pinMode(MOTOR_5_PWM_PIN, OUTPUT);
  pinMode(MOTOR_5_DIR_PIN, OUTPUT);
  analogWrite(MOTOR_5_PWM_PIN, 0);
  digitalWrite(MOTOR_5_DIR_PIN, LOW);
}

/****************************************************************************
 Function: set_pwm_frequency
 Description: Set PWM frequency for motor control
****************************************************************************/
void set_pwm_frequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

/****************************************************************************
 Function: serialEvent
 Description: Handle incoming serial data from Processing
****************************************************************************/
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}