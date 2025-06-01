// TODO header comment

#include <Encoder.h>

const double PI             = 3.1415926535;
const double RAD_PER_REV    = 2 * PI;
const float  COUNTS_PER_REV = 48.0;
const float  GEAR_RATIO     = 4.4; // For computing the angle

// Define PWM and DIR output pins for the motor
const int MOTOR_1_PWM_PIN = 5;
const int MOTOR_1_DIR_PIN = 8;
const int MOTOR_5_PWM_PIN = 6;
const int MOTOR_5_DIR_PIN = 7;

Encoder encT1(2,3); // First encoder, for theta1




/* MOTORS */



// For setting PWM frequency
const int PWM_DIVIDER = 1;
const int PRESCALE_FACTOR = 64 / PWM_DIVIDER;

// For computing torques ***OPTION 1. REQUIRES EXPERIMENTAL TUNING***
#define TORQUE_DIVISOR (3*0.0183) // Based on 3x stall torque for our motors. 

// For computing torques ***OPTION 2. Theoretical, but not sure how accurate***
static const double Tstall_max = 51.975245; // [N-mm]. From motor data sheet.

// For reading encoder counts from follower
typedef union {
  int integer;
  byte binary[2];
} BinaryIntUnion; 

BinaryIntUnion counts5Union;
static int previous_counts5 = 0;
static int counts5 = 0;

// For mapping to motor angles
static int counts1;
static double motorAngle1, motorAngle5; // [rad]. For use in computation
static double motorAngle1_deg, motorAngle5_deg; // [º]. For printing only

/* MOTORS (End) */





/* USER POSITION */

// For mapping motor angles to Theta1 and Theta5
static const double r_pulley = 5.1, r_sector = 75.0; // [mm]. Capstan drive radii for computations. Same for both angles
//static const double r_pulley = 1.0, r_sector = 1.0; // TESTING ONLY

#define STARTER_ANGLE 14 // Modify for tuning/calibration

#define THETA_1_START (STARTER_ANGLE*PI/180) // Starting angle for motor 1
#define THETA_5_START ((180-STARTER_ANGLE)*PI/180) // Starting angle for motor 5
static double theta1, theta5;

// Link lengths (per CAD)
static const double a1 = 127.0; // [mm]
static const double a2 = 152.4; // [mm]
static const double a3 = 152.4; // [mm]. Same as a2.
static const double a4 = 127.0; // [mm]. Same as a1.
static const double a5 = 0.0;

// Pantograph kinematics
static double P2x, P2y, P4x, P4y, mag4from2, mag2fromH, Phx, Phy, mag3fromH, P3x, P3y;

// User speed
static double P3x_prev, P3x_prev2, dP3x, dP3x_prev, dP3x_prev2, dP3x_filt, dP3x_filt_prev, dP3x_filt_prev2; // For x direction
static double P3y_prev, P3y_prev2, dP3y, dP3y_prev, dP3y_prev2, dP3y_filt, dP3y_filt_prev, dP3y_filt_prev2; // For y direction

/* USER POSITION (End) */



/* FORCES AND TORQUES */ 

// Jacobian
static double d, b, h; // Magnitude variables
static double d1x2, d1y2, d5x4, d5y4, d1y4, d1x4, d5y2, d5x2; // Joint coordinate derivatives
static double d1d, d5d, d1b, d5b, d1h, d5h; // Magnitude variable derivatives
static double d1yh, d1xh, d5yh, d5xh; // Point h coordinate derivatives
static double d1y3, d5y3, d1x3, d5x3; // User coordinate derivatives

static double Fx, Fy; // Forces
static double T1, T5, Tmotor1, Tmotor5; // Torques

/* FORCES AND TORQUES (End) */ 


/* RENDERING */

// General

// WITHIN panto
// #define ARD_X_MIN -150 // [mm]. Min X position of Arduino. To map to min width pixels (0) in Processing
// #define ARD_X_MAX 150 // [mm]. Max X position of Arduino. To map to max width pixels in Processing
// #define ARD_Y_MIN 110 // [mm]. Min X position of Arduino. To map to min height pixels (0) in Processing
// #define ARD_Y_MAX 280 // [mm]. Max Y position of Arduino. To map to max height pixels in Processing

// OUTSIDE panto
#define ARD_X_MIN -80 // [mm]. Min X position of Arduino. To map to min width pixels (0) in Processing
#define ARD_X_MAX 80 // [mm]. Max X position of Arduino. To map to max width pixels in Processing
#define ARD_Y_MIN 110 // [mm]. Min X position of Arduino. To map to min height pixels (0) in Processing
#define ARD_Y_MAX 280 // [mm]. Max Y position of Arduino. To map to max height pixels in Processing

// NOTE: these can be DIFF than pantograph limits
//      --These define XY limits of window, which correspond to pixel limits.
//      --Graphics then map to the XY positions in window which correspond to their pixels
//      --Pantograph then moves within or outside of these. Will still hit obstacles at right places, since these map to XY frame, and panto moves in same frame.
//      --Just need to be sure to match this w limits in Processing.

// MY DISPLAY PARAMETERS
// #define WIDTH_PIXELS 1440 // Max width of display in pixels. Corresponds to ARD_X_MAX
// #define HEIGHT_PIXELS 900 // Max height of display in pixels. Corresponds to ARD_Y_MAX

// FINAL DISPLAY PARAMETERS
#define WIDTH_PIXELS 1512 // Max width of display in pixels. Corresponds to ARD_X_MAX
#define HEIGHT_PIXELS 982 // Max height of display in pixels. Corresponds to ARD_Y_MAX

// FLOORS
float Level0_y = round(map(130, 125, 275, HEIGHT_PIXELS, 0)); // [pixels]
float Level1_y = round(map(180, 125, 275, HEIGHT_PIXELS, 0)); // [pixels]
float Level2_y = round(map(200, 125, 275, HEIGHT_PIXELS, 0)); // [pixels]
float Level3_y = round(map(240, 125, 275, HEIGHT_PIXELS, 0)); // [pixels]

float Level0_x = round(map(150-30, 0, 300, 0, WIDTH_PIXELS)); // [pixels]
float Level1_x = 0;                                           // [pixels]
float Level2_x = round(map(150+30, 0, 300, 0, WIDTH_PIXELS)); // [pixels]
float Level3_x = round(map(150-80, 0, 300, 0, WIDTH_PIXELS)); // [pixels]
  
#define LEVEL_HEIGHT 80 // [pixels]
#define OTHER_LEVEL_WIDTH 520 // [pixels]
float Level0_width = round(map(60, 0, 300, 0, WIDTH_PIXELS)); // [pixels]

float floor0_x1 = map(Level0_x, 0, WIDTH_PIXELS, ARD_X_MIN, ARD_X_MAX); // [mm]. From pixels.
float floor0_y1 = map(Level0_y, 0, HEIGHT_PIXELS, ARD_Y_MAX, ARD_Y_MIN); // [mm]. From pixels.
float floor0_x2 = floor0_x1 + map(Level0_width, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // [mm]. From pixels.
float floor0_y2 = floor0_y1 - map(LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX - ARD_Y_MIN) ); // [mm]. From pixels.

float floor1_x1 = map(Level1_x, 0, WIDTH_PIXELS, ARD_X_MIN, ARD_X_MAX); // [mm]. From pixels.
float floor1_y1 = map(Level1_y, 0, HEIGHT_PIXELS, ARD_Y_MAX, ARD_Y_MIN); // [mm]. From pixels.
float floor1_x2 = floor1_x1 + map(OTHER_LEVEL_WIDTH + 50, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // [mm]. From pixels.
float floor1_y2 = floor1_y1 - map(LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Map height in pixels 

float floor2_x1 = map(Level2_x, 0, WIDTH_PIXELS, ARD_X_MIN, ARD_X_MAX); // [mm]. From pixels.
float floor2_y1 = map(Level2_y, 0, HEIGHT_PIXELS, ARD_Y_MAX, ARD_Y_MIN); // [mm]. From pixels.
float floor2_x2 = floor2_x1 + map(OTHER_LEVEL_WIDTH , 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // Map width in pixels
float floor2_y2 = floor2_y1 - map(LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Map height in pixels 

float floor3_x1 = map(Level3_x, 0, WIDTH_PIXELS, ARD_X_MIN, ARD_X_MAX); // [mm]. From pixels.
float floor3_y1 = map(Level3_y, 0, HEIGHT_PIXELS, ARD_Y_MAX, ARD_Y_MIN); // [mm]. From pixels.
float floor3_x2 = floor3_x1 + map(OTHER_LEVEL_WIDTH , 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // Map width in pixels
float floor3_y2 = floor3_y1 - map(LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Map height in pixels

#define WALL_WIDTH 10 // [pixels]

float floor4_x1 = ARD_X_MIN - map(10*LEVEL_HEIGHT, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN)); // [mm]. From pixels. Extend left to ensure render
float floor4_y1 = ARD_Y_MAX + map(10*LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX - ARD_Y_MIN)); // [mm]. From pixels. Extend up to ensure render
float floor4_x2 = ARD_X_MIN + map(WALL_WIDTH, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN)); // [mm]. From pixels.
float floor4_y2 = ARD_Y_MIN - map(10*LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX - ARD_Y_MIN)); // [mm]. From pixels. Extend down to ensure render

float floor5_x1 = ARD_X_MIN + map(WALL_WIDTH, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN)); // [mm]. From pixels. Near top left corner
float floor5_y1 = ARD_Y_MAX + map(10*LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX - ARD_Y_MIN)); // [mm]. From pixels. Extend up to ensure render
float floor5_x2 = ARD_X_MAX + map(10*LEVEL_HEIGHT, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN)); // [mm]. From pixels. Extend right to ensure render
float floor5_y2 = ARD_Y_MAX - map(WALL_WIDTH, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // [mm]. From pixels.

float floor6_x1 = ARD_X_MAX - map(WALL_WIDTH , 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // [mm]. From pixels. Near top left corner
float floor6_y1 = ARD_Y_MAX - map(WALL_WIDTH, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // [mm]. From pixels. Near top left corner
float floor6_x2 = ARD_X_MAX + map(10*LEVEL_HEIGHT, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN)); // [mm]. From pixels. Extend right to ensure render
float floor6_y2 = ARD_Y_MIN - map(10*LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX - ARD_Y_MIN)); // [mm]. From pixels. Extend down to ensure render

float floor7_x1 = ARD_X_MIN + map(WALL_WIDTH, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN)); // [mm]. Near bottom left corner
float floor7_y1 = ARD_Y_MIN + map(WALL_WIDTH, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // [mm]. From pixels. Near bottom left corner
float floor7_x2 = ARD_X_MAX - map(WALL_WIDTH , 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // [mm]. From pixels. Near bottom right corner
float floor7_y2 = ARD_Y_MIN - map(10*LEVEL_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX - ARD_Y_MIN)); // [mm]. From pixels. Extend down to ensure render


double kwall = 1; //[N/mm]

typedef enum{ // For entry direction determination
  LeftSide,
  RightSide,
  TopSide,
  BottomSide
} SideType_t;
bool currently_InObstacle = false; // Initialize to false
bool previously_InObstacle = false; // Initialize to false
SideType_t WallentrySide;


/* SPECIAL RENDERS */
bool currently_InSpecial = false; // Initialize to false

// COBWEBS
double bweb = 0.0003; // [N-s/mm]. Previously 0.00025

// #define COBWEB_WIDTH 200 // [pixels]
// #define COBWEB_HEIGHT 200 // [pixels]
#define SPACING 75 // [pixels]
#define REGION_HEIGHT 150 // [pixels]
float region_width = OTHER_LEVEL_WIDTH - 2*SPACING; // [pixels]

float cobweb_x1 = floor2_x1 + map(SPACING, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) );  // Map top left corner of rectangle
float cobweb_y1 = floor2_y1 + map(REGION_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) );
float cobweb_x2 = cobweb_x1 + map(region_width, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // Map width in pixels
float cobweb_y2 = cobweb_y1 - map(REGION_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Map height in pixels

// WIND
#define WIND_FORCE 2.0 // [N]
double bwind = 0.00025; // [N-s/mm]

float wind_x1 = floor1_x1 + map(SPACING+50, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) );  // Map top left corner of rectangle
float wind_y1 = floor1_y1 + map(REGION_HEIGHT+130, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) );
float wind_x2 = wind_x1 + map(region_width, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // Map width in pixels
float wind_y2 = wind_y1 - map(REGION_HEIGHT+80, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Map height in pixels

// ICE
double bice = 0.00075; // [N-s/mm]. Previously 0.00015

float ice_x1 = floor3_x1 + map(SPACING, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) );  // Map top left corner of rectangle
float ice_y1 = floor3_y1 + map(REGION_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) );
float ice_x2 = ice_x1 + map(region_width, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // Map width in pixels
float ice_y2 = ice_y1 - map(REGION_HEIGHT, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Map height in pixels

// TRAMPOLINES
#define TRAMPOLINE_FX 1.5 // [N]
#define TRAMPOLINE_FY 3.0 // [N]

float trampoline1_x1 = floor0_x1 + map((2*Level0_width)/3, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // [mm]. From pixels.  // Map top left corner of rectangle
float trampoline1_y1 = floor0_y1 + map(REGION_HEIGHT+10, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) );
float trampoline1_x2 = trampoline1_x1 + map(region_width, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // Map width in pixels
float trampoline1_y2 = floor0_y1 + map(10, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) );

bool Trampoline1Flag = false; // Initialize to false


float trampoline2_x1 = floor1_x1; // [mm]. Left side of Floor 1
float trampoline2_y1 = floor1_y1 + map(REGION_HEIGHT+10, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Above Floor 1 by REGION_HEIGHT + 10 pixels
float trampoline2_x2 = trampoline2_x1 + map(Level0_width/5, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // Map width in pixels
float trampoline2_y2 = floor1_y1 + map(10, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Above Floor 1 by 10 pixels

bool Trampoline2Flag = false; // Initialize to false


float trampoline3_x1 = floor2_x1 + map(region_width+10+SPACING, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // [mm]. From pixels. Right side of Floor 2
float trampoline3_y1 = floor2_y1 + map(REGION_HEIGHT+10, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Above Floor 2 by REGION_HEIGHT + 10 pixels
float trampoline3_x2 = trampoline3_x1 + map(Level0_width/5, 0, WIDTH_PIXELS, 0, (ARD_X_MAX - ARD_X_MIN) ); // Map width in pixels
float trampoline3_y2 = floor2_y1 + map(10, 0, HEIGHT_PIXELS, 0, (ARD_Y_MAX-ARD_Y_MIN) ); // Above Floor 2 by 10 pixels

bool Trampoline3Flag = false; // Initialize to false

/* SPECIAL RENDERS (End) */


/* RENDERING (End) */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize motors
  initMotors();

  // Configure Encoder pins
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  

  // Initialize encoder counts
  //*** Assume starting at theta1 = 0º. WILL HAVE TO CALIBRATE in the future
  encT1.write(counts1);

  // Initilalize previous variables
  P3x_prev = 0; // x direction
  P3x_prev2 = 0;
  dP3x_prev = 0;
  dP3x_prev2 = 0;
  dP3x_filt_prev = 0;
  dP3x_filt_prev2 = 0;
  P3y_prev = 0; // y direction
  P3y_prev2 = 0;
  dP3y_prev = 0;
  dP3y_prev2 = 0;
  dP3y_filt_prev = 0;
  dP3y_filt_prev2 = 0;

}

void loop() {
  // put your main code here, to run repeatedly:

  getMotorAngles(); // Update motor angle varibles
  getUserPos(); // [mm]. Update XY values for user position
  getUserSpeed(); // [mm/s]. Update X and Y velocity values
  getJacobian(); // Update Jacobian intermediates and entries for subsequent force computation
  getForces(); // Compute forces based on user position and required motor torques
  updateDutyCycles(); // Change duty cycles to achieve calculated motor torques

  resetPrevVals(); // Reset trackers for previous values. Should be done at end of loop.

  // PRINTS FOR DEBUGGIN ONLY
  // printLRvals(floor2_x1,floor2_y1);
  // printLRvals(floor2_x2,floor2_y2);

}

/****************************************************************************
 Function
     getMotorAngles
 Description
     Get angles of motor shafts
 Notes
     Use as own function to ensure proper angle values from encoders
     separately from position computation.
****************************************************************************/
void getMotorAngles(void)
{
  /* ENCODER 1 */
  counts1 = encT1.read(); // Read encoder counts

  motorAngle1 = (counts1/COUNTS_PER_REV/GEAR_RATIO)*RADS_PER_REV; // [rads]
  motorAngle1_deg = motorAngle1 * 180/PI;

  /* ENCODER 1 (End) */



  /* ENCODER 5 */
  receive_remote_arduino(); // Get counts for encoder 5 from follower

  motorAngle5 = (counts5/COUNTS_PER_REV/GEAR_RATIO)*RADS_PER_REV; // [rads]
  motorAngle5_deg = motorAngle5 * 180/PI;

  /* ENCODER 5 (End) */

  //printLRvals(motorAngle5_deg, motorAngle1_deg); // Debugging only. Comment out  later
  
}

/****************************************************************************
 Function
     getUserPos
 Description
     Get user positions using pantograph kinematics
****************************************************************************/
void getUserPos(void)
{

  // // Test angles
  // theta1 = -1*( (r_pulley/r_sector) * motorAngle1);
  // theta5 = -1*( (r_pulley/r_sector) * motorAngle5);
  // printLRvals( (theta5*180/PI) , (theta1*180/PI) ); // [º] Debugging only. Comment out  later

  // /* Get Theta1 and Theta5 [rads] */
  theta1 = THETA_1_START -1*((r_pulley/r_sector)*motorAngle1); // Negative sign b/c motor rotates opposite direction of link1
  theta5 = THETA_5_START - ( (r_pulley/r_sector)*(motorAngle5)); // Theta5 should be π (180º) when motorAngle5 is zero (on start)
  // From there, increasing motorAngle5 decreases theta5
  //printLRvals( (theta5*180/PI) , (theta1*180/PI) ); // [º] Debugging only. Comment out  later

  /* Get P2 and P4 [mm] */
  P2x = a1*cos(theta1); // Per kinematics paper
  P2y = a1*sin(theta1);
  P4x = a4*cos(theta5) - a5;
  P4y = a4*sin(theta5);
  //printLRvals(P2x,P2y); //Debugging only
  //printLRvals(P4x,P4y); //Debugging only

  /* Get magnitudes [mm] */
  mag4from2 = sqrt( (P4x-P2x)*(P4x-P2x) + (P4y-P2y)*(P4y-P2y)  ); // Per basic geometry. Manual squaring.
  //Note mag2from4 must be identical
  mag2fromH = ( (a2*a2) - (a3*a3) + (mag4from2*mag4from2) )/(2*mag4from2); // Per kinematics paper. Manual squaring.
  //Note magHfrom2 must be identical
  //printLRvals(mag4from2,mag2fromH); //Debugging only

  /* Get Ph [mm] */
  Phx = P2x + (mag2fromH/mag4from2)*(P4x-P2x);
  Phy = P2y + (mag2fromH/mag4from2)*(P4y-P2y);
  //printLRvals(Phx,Phy); //Debugging only

  /* Get P3x and P3y [mm] */
  mag3fromH = sqrt( (a2*a2) - (mag2fromH*mag2fromH) ); // Manual squaring
  //printLRvals( mag2fromH, a2 ); //Debugging only
  P3x = Phx + (mag3fromH/mag4from2)*(P4y-P2y);
  P3y = Phy - (mag3fromH/mag4from2)*(P4x-P2x);

  printLRvals(P3x, P3y); // LEAVE IN TO SEND TO PROCESSING.
}

/****************************************************************************
 Function
     getUserSpeeed
 Description
     Get user x and y velocities with filtered time-based derivative estimation
****************************************************************************/
void getUserSpeed(void)
{
  /* X-DIRECTION */
  // Calculate velocity with loop time estimation
  dP3x = (double)(P3x - P3x_prev) / 0.001; // [mm/s]

  // Calculate the filtered velocity of the handle
  //using an infinite impulse response filter
  dP3x_filt = 0.9*dP3x + 0.1*dP3x_prev;
  
  /* X-DIRECTION (End) */

  /* Y-DIRECTION */
  // Calculate velocity with loop time estimation
  dP3y = (double)(P3y - P3y_prev) / 0.001; // [mm/s]

  // Calculate the filtered velocity of the handle
  //using an infinite impulse response filter
  dP3y_filt = 0.9*dP3y + 0.1*dP3y_prev;
    
  /* Y-DIRECTION (End) */

  //printLRvals(dP3x_filt, dP3y_filt);
}






/****************************************************************************
 Function
     getJacobian
 Description
     Get Jacobian for force computations
****************************************************************************/
void getJacobian(void)
{
  /* JACOBIAN NOTES */
  // J = [ dx3/dt1, dx3/dt5; dy3/dt1, dy3/dt5] = [d1x3, d5x3; d1y3, d5y3]
  // Note for simplicity, dizj = dzj/dti = deriv of jth var wrt theta i

  /* COMPUTE JACOBIAN INTERMEDIATES */
  // Update d, b, and h according to magnitude vars from position computations
  d = mag4from2; // [mm]. Same as mag2from4
  b = mag2fromH; // [mm]. Same as magHfrom2
  h = mag3fromH; // [mm]. Same as magHfrom3

  // Joint coordinate derivatives [mm/rad, but rad dimensionless, so mm]
  d1x2 = -a1*sin(theta1);
  d1y2 = a1*cos(theta1);
  d5x4 = -a4*sin(theta5);
  d5y4 = a4*cos(theta5);
  d1y4 = 0.0;
  d1x4 = 0.0;
  d5y2 = 0.0;
  d5x2 = 0.0;

  // Magnitude derivatives [mm]
  d1d = ( (P4x-P2x)*(d1x4-d1x2) + (P4y-P2y)*(d1y4-d1y2) )/d; // d
  d5d = ( (P4x-P2x)*(d5x4-d5x2) + (P4y-P2y)*(d5y4-d5y2) )/d;

  d1b = d1d - d1d * ( (a2*a2) - (a3*a3) + (d*d) )/( 2 * d * d ); // b. Manual squaring.
  d5b = d5d - d5d * ( (a2*a2) - (a3*a3) + (d*d) )/( 2 * d * d ); // Manual squaring.

  d1h = ( -b*d1b )/h; // h
  d5h = ( -b*d5b )/h;

  // Point h coordinate derivatives [mm]
  d1yh = d1y2 + (d1b*d - d1d*b)*(P4y - P2y)/(d*d) + (b/d)*(d1y4 - d1y2); // Manual squaring
  d5yh = d5y2 + (d5b*d - d5d*b)*(P4y - P2y)/(d*d) + (b/d)*(d5y4 - d5y2); // Manual squaring

  d1xh = d1x2 + (d1b*d - d1d*b)*(P4x - P2x)/(d*d)  + (b/d)*(d1x4 - d1x2); // Manual squaring
  d5xh = d5x2 + (d5b*d - d5d*b)*(P4x - P2x)/(d*d)  + (b/d)*(d5x4 - d5x2); // Manual squaring

  /* COMPUTE JACOBIAN ENTRIES [mm] */
  d1y3 = d1yh - (h/d)*(d1x4-d1x2) - (d1h*d - d1d*h)/(d*d) * (P4x - P2x); // Manual squaring
  d5y3 = d5yh - (h/d)*(d5x4-d5x2) - (d5h*d - d5d*h)/(d*d) * (P4x - P2x); // Manual squaring

  d1x3 = d1xh + (h/d)*(d1y4-d1y2) + (d1h*d - d1d*h)/(d*d)*(P4y - P2y); // Manual squaring
  d5x3 = d5xh + (h/d)*(d5y4-d5y2) + (d5h*d - d5d*h)/(d*d)*(P4y - P2y); // Manual squaring
}

/****************************************************************************
 Function
     getForces
 Description
     Calculate updated forces and corresponding motor torques
****************************************************************************/
void getForces(void)
{
  /* FORCES */
  //***Depends on graphics and obstacles. TBD.
  // Fx = 0; // [N].
  // Fy = 0; // [N]. So, for now, just set to zero.

  // Let's try a REALLY soft vertical wall at 200 mm
  // double ktester = 0.75; // [N/mm].
  // double y_testwall = 185.0; // [mm]
  // if(P3y <= y_testwall)
  // {
  //   Fy = 0;
  // } else {
  //   Fy = -ktester*(P3y-y_testwall);
  // }

  /* WALLS */

  CheckInWall(floor0_x1, floor0_y1, floor0_x2, floor0_y2); // Floor 0
  CheckInWall(floor1_x1, floor1_y1, floor1_x2, floor1_y2); // Floor 1
  CheckInWall(floor2_x1, floor2_y1, floor2_x2, floor2_y2); // Floor 2
  CheckInWall(floor3_x1, floor3_y1, floor3_x2, floor3_y2); // Floor 3
  CheckInWall(floor4_x1, floor4_y1, floor4_x2, floor4_y2); // Floor 4
  CheckInWall(floor5_x1, floor5_y1, floor5_x2, floor5_y2); // Floor 5
  CheckInWall(floor6_x1, floor6_y1, floor6_x2, floor6_y2); // Floor 6
  CheckInWall(floor7_x1, floor7_y1, floor7_x2, floor7_y2); // Floor 7

  CheckCobwebs();
  CheckWind();
  CheckIce();

  CheckTrampoline1();
  CheckTrampoline2();
  CheckTrampoline3();

  // DEBUGGING PRINTS ONLY
  // Serial.print(WallentrySide);
  // Serial.print(", ");
  // Serial.print(Fx);
  // Serial.print(", ");
  // Serial.println(Fy);
  

  /* WALLS (End) */

  // OBSTACLE PROCESSING
  if(Trampoline1Flag) // If user hit trampoline 1
  {
    // First, ensure we havent hit outside walls. If so, cancel trampoline
    if( (P3x <= floor4_x2) || (P3x >= floor6_x2) )
    {
      Trampoline1Flag = false;
      // Don't alter forces. These will be rendered by wall checks.
    } else if(P3y < floor1_y1) // If user still below Floor 1
    {
      // Render a constant force pushing them up above Floor 1
      Fx = -1*TRAMPOLINE_FX; // [N]
      Fy = TRAMPOLINE_FY; // [N]
    } else {
      // User has crossed Floor 1. Clear forces and flag
      Fx = 0;
      Fy = 0;
      Trampoline1Flag = false;
    }
  } else if (Trampoline2Flag) // If user hit Trampoline 2
  {
    // First, ensure we haven't crossed passed Floor 3 laterally. If so, cancel
    if( P3x >= floor3_x1 )
    {
      Trampoline2Flag = false;
      // Don't alter forces. These will be rendered by wall checks.
    } else if(P3y < floor3_y1) // If user still below floor 3
    {
      // Render a constant force pushing them up above Floor 3
      Fx = 0.6*TRAMPOLINE_FX; // [N]
      Fy = 1.25*TRAMPOLINE_FY; // [N]
    } else {
      // User has crossed Floor 3. Clear forces and flag.
      Fx = 0;
      Fy = 0;
      Trampoline2Flag = false;
    }
  } else if (Trampoline3Flag) // If user hit Trampoline 3
  {
    // First, ensure we haven't crossed passed Floor 3 laterally. If so, cancel
    if( P3x <= ((floor3_x1 + floor3_x2)/2) )
    {
      Trampoline3Flag = false;
      // Don't alter forces. These will be rendered by wall checks.
    } else if(P3y < floor3_y1) // If user still below floor 3
    {
      // Render a constant force pushing them up above Floor 3
      Fx = -0.6*TRAMPOLINE_FX; // [N]
      Fy = 1.25*TRAMPOLINE_FY; // [N]
    } else {
      // User has crossed Floor 3. Clear forces and flag.
      Fx = 0;
      Fy = 0;
      Trampoline3Flag = false;
    }
  } else if( (!currently_InObstacle) && (!currently_InSpecial) ) // If, after ALL CHECKS, we weren't in an obstacle or special region
  {
    Fx = 0;
    Fy = 0; // Clear forces
  }
  previously_InObstacle = currently_InObstacle; // Reset InObstacle trackers.
  currently_InObstacle = false; // Current should default to false.
  currently_InSpecial = false;
  // Will be checked again on next loop and reset to true if still in obstacle.

}

/****************************************************************************
 Function
     updateDutyCycles
 Description
     Calculate motor torques and update duty cycles
****************************************************************************/
void updateDutyCycles(void)
{
  /* TORQUES */
  // Torques = (Jacobian TRANSPOSE) * Forces
  // From above, Jacobian is J = [ dx3/dt1, dx3/dt5; dy3/dt1, dy3/dt5] = [d1x3, d5x3; d1y3, d5y3]
  // So, Jacobian TRANSPOSE is J^T = [d1x3, d1y3; d5x3, d5y3]
  // Therefore
  // [T1; T5] = [d1x3, d1y3; d5x3, d5y3]*[Fx; Fy]
  // Therefore
  T1 = d1x3*Fx + d1y3*Fy; // [N-mm]
  T5 = d5x3*Fx + d5y3*Fy; // [N-mm]
  
  // But, these are JOINT torques at end of capstan drive
  // To achieve these with motor, must map to motor torque based on capstan radii
  // Tmotori/r_pulley = Ti/r_sector;
  // Therefore
  Tmotor1 = (r_pulley/r_sector)*T1; // [N-mm]
  Tmotor5 = (r_pulley/r_sector)*T5; // [N-mm]

  //printLRvals(Tmotor5,Tmotor1); // For debugging. Comment out later.

  // NOTE: Torques are currently in units of N-mm, NOT N-m.

  /* TORQUES (End) */



  // Variables used for each motor
  float duty;
  unsigned int output;



  /* MOTOR 1 */

  // Determine correct direction for motor torque
  if(Tmotor1 > 0) { 
    digitalWrite(MOTOR_1_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_1_DIR_PIN, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  //duty = sqrt(abs(Tmotor1)/TORQUE_DIVISOR); // ***OPTION 1***
  duty = abs(Tmotor1)/Tstall_max; // ***OPTION 2***

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);     // convert duty cycle to output signal
  analogWrite(MOTOR_1_PWM_PIN, output);                // output the signal

  /* MOTOR 1 (End) */



  /* MOTOR 5 */

  // Determine correct direction for motor torque
  if(Tmotor5 > 0) { 
    digitalWrite(MOTOR_5_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_5_DIR_PIN, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  //duty = sqrt(abs(Tmotor5)/TORQUE_DIVISOR); // ***OPTION 1***
  duty = abs(Tmotor5)/Tstall_max; // ***OPTION 2***

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);     // convert duty cycle to output signal
  analogWrite(MOTOR_5_PWM_PIN, output);                // output the signal

  /* MOTOR 5 (End) */

}



/****************************************************************************
 Function
     ResetPreviousVals
 Description
     Resets variables for storing previous vals. Should be done at end of loop
****************************************************************************/
void resetPrevVals(void)
{

  // Reset the prior position and velocity
  P3x_prev2 = P3x_prev;
  P3x_prev = P3x;

  dP3x_prev2 = dP3x_prev;
  dP3x_prev = dP3x;

  dP3x_filt_prev2 = dP3x_filt_prev;
  dP3x_filt_prev = dP3x_filt;



  // Reset the prior Y position and velocity
  P3y_prev2 = P3y_prev;
  P3y_prev = P3y;

  dP3y_prev2 = dP3y_prev;
  dP3y_prev = dP3y;

  dP3y_filt_prev2 = dP3y_filt_prev;
  dP3y_filt_prev = dP3y_filt;
}











/**************** HELPERS (Not called in main loop) ***********************/

/****************************************************************************
 Function
     receive_remote_arduino
 Description
     Get encoder counts from follower Arduino
****************************************************************************/

void receive_remote_arduino(void) {  
  
  // read our follower position
  if (Serial.available() > 1){                  // if there is at least 2 bytes of data to read from the serial
    previous_counts5 = counts5;                 // backup old follower position
    Serial.readBytes(counts5Union.binary, 2);   // read the bytes in
    counts5 = counts5Union.integer;             // Save the integer
    if (isnan(counts5))                                                // if corrupt, just use the old value
      counts5 = previous_counts5;
    Serial.flush(); // flush the serial for good measure
  }
}

/****************************************************************************
 Function
     printLRVals
 Description
     helper to simply print two requested angles for use in debugging
****************************************************************************/
void printLRvals(double left, double right)
{
  /* Prints for checking values */
  Serial.print(left);
  Serial.print(" ");
  Serial.println(right);
  /* Prints for checking values */
}

/****************************************************************************
 Function
     initMotors
 Description
     helper to initialize BOTH motors
****************************************************************************/
void initMotors() {

  /* MOTOR 1 */

  // Set PWM frequency 
  set_pwm_frequency(MOTOR_1_PWM_PIN, PWM_DIVIDER); 
  
  // Output pins
  pinMode(MOTOR_1_PWM_PIN, OUTPUT);  // PWM pin for motor 1
  pinMode(MOTOR_1_DIR_PIN, OUTPUT);  // dir pin for motor 1
  
  // Initialize motor 
  analogWrite(MOTOR_1_PWM_PIN, 0);     // set to not be spinning (0/255)
  digitalWrite(MOTOR_1_DIR_PIN, LOW);  // set direction

  /* MOTOR 1 (End) */



  /* MOTOR 5 */

  // Set PWM frequency 
  set_pwm_frequency(MOTOR_5_PWM_PIN, PWM_DIVIDER); 
  
  // Output pins
  pinMode(MOTOR_5_PWM_PIN, OUTPUT);  // PWM pin for motor 5
  pinMode(MOTOR_5_DIR_PIN, OUTPUT);  // dir pin for motor 5
  
  // Initialize motor 
  analogWrite(MOTOR_5_PWM_PIN, 0);     // set to not be spinning (0/255)
  digitalWrite(MOTOR_5_DIR_PIN, LOW);  // set direction

  /* MOTOR 5 (End) */
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
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

/***************************** RENDERING HELPERS *********************************/


/****************************************************************************
 Function
     CheckInWall
 Description
     helper to determine if inside a given floor
****************************************************************************/

void CheckInWall(float top_left_x, float top_left_y, float bottom_right_x, float bottom_right_y)
{
  if( (P3x >= top_left_x) && (P3x <= bottom_right_x) && (P3y <= top_left_y) && (P3y >= bottom_right_y) )
  {
    //Serial.println("In obstacle");
    currently_InObstacle = true; // Since now in a region, reflect this in module-level tracker.

    // If not PREVIOUSLY in region, determine entry side
    if(!previously_InObstacle)
    {
      if ( (P3y_prev < P3y) && (P3y_prev <= bottom_right_y)) // if moving up AND previously below wall bottom
      {
        WallentrySide = BottomSide;
        //Serial.println("Bottom");
      } else if ( (P3y_prev >= P3y) && (P3y_prev >= top_left_y) ) // if moving down AND previously above wall top
      {
        WallentrySide = TopSide; // Moving down, so entering from top
        //Serial.println("Top");
      } else if ( (P3x_prev < P3x) && (P3x_prev <= top_left_x) ) // if moving right AND previously outside wall left
      {
        WallentrySide = LeftSide;
      } else if ( (P3x_prev >= P3x) && (P3x_prev >= bottom_right_x) ) // if moving left AND previously outside wall right
      {
        WallentrySide = RightSide;
      }
    }

    // Use entry side to set force
    switch(WallentrySide){
      case TopSide: // Apply force up
      {
        Fx = 0;
        Fy = -kwall*(P3y - top_left_y);
      }
      break;

      case BottomSide: // Apply force down
      {
        Fx = 0;
        Fy = -kwall*(P3y - bottom_right_y);

      }
      break;

      case LeftSide: // Apply force right
      {
        Fx = -kwall*(P3x - top_left_x);
        Fy = 0;
      }
      break;

      case RightSide:
      {
        Fx = -kwall*(P3x - bottom_right_x);
        Fy = 0;
      }
      break;

      default:
        ;   // Do nothing
    }
  }
  // else
  // {
  //   // If get here, not in a wall.
  //   currently_InObstacle = false; // So, not currently in a region. Reflect this in module-level tracker.

  //   Fx = 0;
  //   Fy = 0; // clear forces
  // }

}

/****************************************************************************
 Function
     CheckCobwebs
 Description
     helper to determine if inside cobwebs
****************************************************************************/

void CheckCobwebs(void)
{
  if( (P3x >= cobweb_x1) && (P3x <= cobweb_x2) && (P3y <= cobweb_y1) && (P3y >= cobweb_y2) )
  {
    //Serial.println("In Cobweb");
    currently_InSpecial = true; // Since now in a region, reflect this in module-level tracker.

    Fx = -bweb*dP3x_filt;
    Fy = -bweb*dP3y_filt;
  }
}

/****************************************************************************
 Function
     CheckWind
 Description
     helper to determine if inside windy region
****************************************************************************/

void CheckWind(void)
{
  if( (P3x >= wind_x1) && (P3x <= wind_x2) && (P3y <= wind_y1) && (P3y >= wind_y2) )
  {
    //Serial.println("In Wind");
    currently_InSpecial = true; // Since now in a region, reflect this in module-level tracker.

    Fy = 0; // No y force

    // Render Wind X force
    // double time = millis(); // Get current time cast to double.
    // time = (time/100000); // Convert to seconds
    // Fx = WIND_FORCE*( 1 - cos((PI/2)*time)*sin(time/2) ) - bwind*dP3x_filt; // based on relative velocity
    // Fx = WIND_FORCE*cos(2*PI*time)*sin(time);
    //printLRvals(time, Fx);

    // WHIRLPOOL
    Fy = -1*WIND_FORCE*cos((PI/(wind_x2-wind_x1))*(P3x-wind_x1) );
    
    double midpoint = (wind_y1 + wind_y2)/2; // Midpoint for x force calc
    if ( P3y >= midpoint ) // Go left
    {
      Fx = -1*WIND_FORCE*sin((PI/(wind_x2-wind_x1))*(P3x-wind_x1) );
      //Fx = -1*(WIND_FORCE/2)*(1 - cos( ((2*PI)/(wind_x2-wind_x1))*(P3x-wind_x1) ) );
    } else // Go right
    {
      Fx = WIND_FORCE*sin((PI/(wind_x2-wind_x1))*(P3x-wind_x1) );
      //Fx = (WIND_FORCE/2)*(1 - cos( ((2*PI)/(wind_x2-wind_x1))*(P3x-wind_x1) ) );
    }

    // // NOTE: Wind should never exert force in oppo direction
    // if(Fx < 0)
    // {
    //   Fx = 0;
    // }
  }
}

/****************************************************************************
 Function
     CheckIce
 Description
     helper to determine if inside ice
****************************************************************************/

void CheckIce(void)
{
  if( (P3x >= ice_x1) && (P3x <= ice_x2) && (P3y <= ice_y1) && (P3y >= ice_y2) )
  {
    //Serial.println("In Ice");
    currently_InSpecial = true; // Since now in a region, reflect this in module-level tracker.

    Fx = bice*dP3x_filt;
    Fy = bice*dP3y_filt;
  }
}

/****************************************************************************
 Function
     CheckTrampoline1
 Description
     helper to determine if inside trampoline 1
****************************************************************************/

void CheckTrampoline1(void)
{
  if( (P3x >= trampoline1_x1) && (P3x <= trampoline1_x2) && (P3y <= trampoline1_y1) && (P3y >= trampoline1_y2) )
  {
    //Serial.println("In Trampoline");
    currently_InSpecial = true; // Since now in a region, reflect this in module-level tracker.

    Trampoline1Flag = true; // Set trampoline 1 flag for continued force rendering
  }
}

/****************************************************************************
 Function
     CheckTrampoline2
 Description
     helper to determine if inside trampoline 2
****************************************************************************/

void CheckTrampoline2(void)
{
  if( (P3x >= trampoline2_x1) && (P3x <= trampoline2_x2) && (P3y <= trampoline2_y1) && (P3y >= trampoline2_y2) )
  {
    //Serial.println("In Trampoline");
    currently_InSpecial = true; // Since now in a region, reflect this in module-level tracker.

    Trampoline2Flag = true; // Set trampoline 1 flag for continued force rendering
  }
}

/****************************************************************************
 Function
     CheckTrampoline3
 Description
     helper to determine if inside trampoline 3
****************************************************************************/

void CheckTrampoline3(void)
{
  if( (P3x >= trampoline3_x1) && (P3x <= trampoline3_x2) && (P3y <= trampoline3_y1) && (P3y >= trampoline3_y2) )
  {
    //Serial.println("In Trampoline");
    currently_InSpecial = true; // Since now in a region, reflect this in module-level tracker.

    Trampoline3Flag = true; // Set trampoline 1 flag for continued force rendering
  }
}