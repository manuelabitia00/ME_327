// Haptic Asteroid Game - Processing Implementation with Pantograph Control
// Team 1: Manuel, Nick, Jorge, Giuse
// AstroTouch - Haptic Space Navigation Game
// ENHANCED VERSION - 3D visuals, improved mechanics, full screen

import processing.serial.*;

// Constants for window dimensions
final int WINDOW_WIDTH      = 600;
final int WINDOW_HEIGHT     = 400;

// Serial settings
final String SERIAL_PORT    = "COM7";
final int    BAUD_RATE      = 9600;

// Target frame rate
final int TARGET_FRAME_RATE = 60;

// Debug flag
final boolean DEBUG_MODE    = true;

Serial myPort;
float xPos_mm, yPos_mm;
float prevXPos_mm, prevYPos_mm;

// Pantograph link lengths & geometry (added)
final float a1   = 127.0;  // mm, link 1 length in pantograph notation
final float a2   = 152.4;  // mm, link 2 length
final float a3   = 152.4;  // mm, link 3 (same length as link 2)
final float a4   = 127.0;  // mm, link 4 (same lenght as link 1)
final float PIVOT_OFFSET    = 0.0;    // mm

// These are the location of the boundaries of our pantograph in real world mm. 
float X_MIN_MM = -43;
float X_MAX_MM =  43;
float Y_MIN_MM =  190;
float Y_MAX_MM =  240;

// This is the spring constant of the outer walls that keep the cursor in the window.
float WALL_SPRING_CONST = 0.008; // N/mm
final float BLACK_HOLE_GRAVITY_CONST = 200; // TODO tune

/****************************************************************************
***************************CODE INSERT START*********************************
*****************************************************************************/

// Game state variables
boolean showPressStart = true;
boolean gameRunning = false;
boolean showLeaderboard = false;
boolean enteringName = false;
boolean deathAnimation = false;
float deathAnimationTimer = 0;
final float DEATH_ANIMATION_DURATION = 1.0; // Shortened to 1 second

// Black hole death animation variables
boolean blackHoleDeath = false;
PVector blackHoleDeathPos = new PVector(0, 0);
PVector ufoDeathPos = new PVector(0, 0);
float ufoDeathSize = 200; // Updated for 200px UFO (was 105, now 200)

// Game variables
int score = 0;
int lives = 5; // Increased to 5 hearts
int maxLives = 5; // Increased to 5 hearts
float gameTime = 0;
int cargoCollected = 0;
float virtualMass = 1.0;
boolean invincible = false;
float invincibleTimer = 0;
float gameSpeedMultiplier = 1.0;
int damageLevel = 0;

// Game objects
ArrayList<Asteroid> asteroids;
ArrayList<Cargo> cargoItems;
ArrayList<BlackHole> blackHoles;
ArrayList<AsteroidFragment> asteroidFragments;
ArrayList<Particle> particles;
ArrayList<Star> stars;
ArrayList<RaceLight> raceLights;
ArrayList<SmokeParticle> smokeParticles;

// Colors and animation
color backgroundColor = color(0, 10, 30);
color ufoColor = color(150, 100, 255);
float ufoSpinAngle = 0;
float frameSpinAngle = 0;

// Wall force field variables
float wallGlowPulse = 0;

// Avatar system
PImage[] avatarImages = new PImage[7];
int currentAvatar = 0; // Index 0-6 for img1-img7

// Leaderboard
int[] highScores = new int[5];
String[] playerNames = {"AAA", "BBB", "CCC", "DDD", "EEE"};
String currentPlayerName = "";
int nameIndex = 0;

// Force rendering constants
final float ASTEROID_IMPACT_FORCE = 8.0;    // N
final float BLACKHOLE_MAX_FORCE = 12.0;     // N
final float CARGO_MASS_INCREMENT = 0.3;     // mass multiplier per cargo
final float BASE_DAMPING = 0.1;             // N⋅s/mm
final float MAX_FORCE = 15.0;               // N - safety limit

// Speed scaling variables
int lastSpeedIncrement = 0;
float baseAsteroidSpeed = 1.0;
float baseCargoSpeed = 3.0;

// Score-based scaling variables
int lastScoreMilestone = 0;
float asteroidSpawnMultiplier = 1.0;
float speedMultiplier = 1.0;

// Velocity calculation for haptics
PVector pantoVelocity = new PVector(0, 0);
PVector previousPantoPos = new PVector(0, 0);

// Asteroid collision force tracking
PVector asteroidCollisionForce = new PVector(0, 0);
long asteroidCollisionStartTime = 0;
final int COLLISION_FORCE_DURATION_MS = 100; // 0.1 seconds in milliseconds

 
/****************************************************************************
***************************CODE INSERT END***********************************
*****************************************************************************/

void settings()
{
  //size(WINDOW_WIDTH, WINDOW_HEIGHT);
  fullScreen();
  
  /****************************************************************************
  ***************************CODE INSERT START*********************************
  *****************************************************************************/
  
  // Enable full screen mode for better immersion
  
  /****************************************************************************
  ***************************CODE INSERT END***********************************
  *****************************************************************************/
  
  
}

void setup()
{
   frameRate(TARGET_FRAME_RATE);

   // Serial init
   myPort = new Serial(this, SERIAL_PORT, BAUD_RATE);
   myPort.bufferUntil('\n');

   if (DEBUG_MODE)
   {
      textSize(16);
      fill(0);
   }
   
 /****************************************************************************
  ***************************CODE INSERT START*********************************
  *****************************************************************************/
  
  // Initialize game objects
  asteroids = new ArrayList<Asteroid>();
  cargoItems = new ArrayList<Cargo>();
  particles = new ArrayList<Particle>();
  blackHoles = new ArrayList<BlackHole>();
  asteroidFragments = new ArrayList<AsteroidFragment>();
  stars = new ArrayList<Star>();
  raceLights = new ArrayList<RaceLight>();
  smokeParticles = new ArrayList<SmokeParticle>();
  
  // Create background stars
  for (int i = 0; i < (width * height) / 8000; i++) {
    stars.add(new Star());
  }
  
  // Load avatar images
  for (int i = 0; i < 7; i++) {
    avatarImages[i] = loadImage("img" + (i + 1) + ".png");
  }
  
  // Initialize leaderboard with default scores (all 0s)
  for (int i = 0; i < 5; i++) {
    highScores[i] = 0;
  }
  
  // Initialize position tracking
  previousPantoPos.set(xPos_mm, yPos_mm);
  
  println("AstroTouch Processing Game Started");
  println("Screen size: " + width + "x" + height + " (Full Screen)");
  println("Pantograph workspace: X(" + X_MIN_MM + " to " + X_MAX_MM + ") Y(" + Y_MIN_MM + " to " + Y_MAX_MM + ")");
   
  /****************************************************************************
  ***************************CODE INSERT END***********************************
  *****************************************************************************/
}

void draw()
{
   background(backgroundColor);

   // Redraw axes every frame
   //stroke(0);
   //line(0, height/2, width,  height/2);
   //line(width/2, 0, width/2, height);

   // Map the realworld coordinates to our window. xPos_mm and yPos_mm are global.
   float xPosWindow = (xPos_mm - X_MIN_MM) * (width  / (X_MAX_MM - X_MIN_MM) );
   float yPosWindow = (yPos_mm - Y_MIN_MM) * (height / (Y_MAX_MM - Y_MIN_MM) );
     
   // Constrain UFO to stay fully within screen bounds (accounting for UFO size)
   float ufoRadius = 120 * 0.5; // UFO diameter 120 pixels (was 200, now 120)
   if (xPosWindow < ufoRadius)
   {
     xPosWindow = ufoRadius;
   }
   else if (xPosWindow > width - ufoRadius)
   {
     xPosWindow = width - ufoRadius;
   }
   
   if (yPosWindow < ufoRadius)
   {
     yPosWindow = ufoRadius;
   }
   else if (yPosWindow > height - ufoRadius)
   {
     yPosWindow = height - ufoRadius;
   }
   
   // Draw the latest position
   //fill(255, 0, 0);
   //noStroke();
   //ellipse(xPosWindow, yPosWindow, 20, 20);

   // Always display real world coords when debugging
   if (DEBUG_MODE)
   {
      fill(255, 255, 0);
      textAlign(LEFT);
      textSize(14);
      text("real world coords (mm) x=" + nf(xPos_mm, 1, 2) + ",  y=" + nf(yPos_mm, 1, 2), 10, height - 40);
      text("window coords (px) x=" + nf(xPosWindow, 1, 1) + ",  y=" + nf(yPosWindow, 1, 1), 10, height - 20);
   }
   
  /****************************************************************************
  ***************************CODE INSERT START*********************************
  *****************************************************************************/
  
  if (showPressStart) {
    updateGameLogic();
    updateStars();
    updateAsteroids();
    updateBlackHoles();
    
    // Spawn asteroids for title screen (25% more frequent)
    if (frameCount % 144 == 0) { // Every 2.4 seconds (was 180, now 144)
      asteroids.add(new Asteroid());
    }
    
    // Spawn black holes for title screen (like gameplay)
    if (frameCount % 450 == 0 && blackHoles.size() < 3) { // Every 7.5 seconds, slower than gameplay
      blackHoles.add(new BlackHole());
    }
    
    drawStars();
    drawBlackHoles();
    drawAsteroids();
    drawSpaceship(xPosWindow, yPosWindow);
    drawPressStartUI();
    drawWallForceField();
  } else if (gameRunning) {
    gameTime += 1.0/60.0;
    
    // Update game logic
    updateGameLogic();
    updateAsteroids();
    updateCargo();
    updateBlackHoles();
    updateParticles();
    updateAsteroidFragments();
    updateStars();
    updateRaceLights();
    updateSmokeParticles();
    checkCollisions(xPosWindow, yPosWindow);
    updateSpeedScaling();
    updateScoreBasedScaling();
    
    // Draw everything
    drawStars();
    drawBlackHoles();
    drawAsteroids();
    drawAsteroidFragments();
    drawCargo();
    drawSpaceship(xPosWindow, yPosWindow);
    drawRaceLights();
    drawParticles();
    drawSmokeParticles();
    drawUI();
    drawWallForceField(); // Draw wall force field on top of everything
    
    // Spawn new objects with score-based scaling
    int asteroidSpawnRate = int(96 / asteroidSpawnMultiplier); // Base 96 frames (25% more frequent), adjusted by score
    if (frameCount % asteroidSpawnRate == 0) {
      asteroids.add(new Asteroid());
    }
    
    if (frameCount % 300 == 0 && cargoItems.size() < 3 && virtualMass < 20.0) {
      cargoItems.add(new Cargo());
    }
    
    if (frameCount % 300 == 0 && blackHoles.size() < 3) {
      blackHoles.add(new BlackHole());
    }
    
    // Update score - 1 point per second, multiplied by cargo collected
    if (frameCount % 60 == 0) {
      int basePoints = 1;
      int cargoMultiplier = max(1, cargoCollected); // At least 1x multiplier
      score += basePoints * cargoMultiplier;
    }
  } else if (deathAnimation) {
    // Death animation state - continue updating physics but show death sequence
    deathAnimationTimer += 1.0/60.0;
    
    updateGameLogic();
    updateParticles();
    updateSmokeParticles();
    updateStars();
    
    drawStars();
    drawBlackHoles();
    drawAsteroids();
    drawCargo();
    
    if (blackHoleDeath) {
      // Black hole death animation - shrink UFO and move toward black hole
      float animationProgress = deathAnimationTimer / DEATH_ANIMATION_DURATION;
      animationProgress = constrain(animationProgress, 0, 1);
      
      // Interpolate UFO position toward black hole center
      ufoDeathPos.lerp(blackHoleDeathPos, animationProgress * 0.1);
      
      // Shrink UFO size
      ufoDeathSize = 200 * (1.0 - animationProgress); // Updated for 200px UFO (was 105, now 200)
      
      // Draw shrinking UFO if it's still visible
      if (ufoDeathSize > 1) {
        drawShrinkingSpaceship(ufoDeathPos.x, ufoDeathPos.y, ufoDeathSize);
      }
    } else {
      // Regular explosion death - don't draw UFO, just show explosion particles
    }
    
    drawParticles();
    drawSmokeParticles();
    drawUI();
    drawWallForceField();
    
    // Show "GAME OVER" text immediately during death animation
    fill(255, 50, 50, 150 + sin(frameCount * 0.2) * 105);
    textAlign(CENTER);
    textSize(90);
    text("GAME OVER", width/2, height/2);
    
    // End death animation and go to game over screen
    if (deathAnimationTimer >= DEATH_ANIMATION_DURATION) {
      deathAnimation = false;
      checkHighScore();
    }
  } else if (enteringName) {
    drawStars();
    drawNameEntry();
    drawWallForceField();
  } else if (showLeaderboard) {
    updateStars();
    drawStars();
    drawLeaderboard();
    drawWallForceField();
  } else {
    drawStars();
    drawGameOver();
    drawWallForceField();
  }
   
  /****************************************************************************
  ***************************CODE INSERT END***********************************
  *****************************************************************************/
}

// This function reads the angles of the Arduino using the serial monitor, computes the xy position of the cursor
// and computes the correct torque to send back to the Arduino. THIS IS THE ONLY FUNCTION WHICH CHANGES THE GLOBAL
// POSITION VARIABLES.
void serialEvent(Serial p)
{
   String in = p.readStringUntil('\n');
   if (in == null)
   {
      return;
   }
   in = in.trim();

   // Incoming cursor packet
   if (in.startsWith("{") && in.endsWith("}"))
   {
      String[] parts = split(in.substring(1, in.length() - 1), ',');
      if (parts.length == 2)
      {
         try
         {
            float theta1Rad = Float.parseFloat(parts[0]);
            float theta5Rad = Float.parseFloat(parts[1]);

            // Compute the position in real world mm from the angles (and also return a lot of other data for the jacobian)
            float[] pos = computeXYfromAngles_mm(theta1Rad, theta5Rad);
            
            // Update previous position for velocity calculation
            previousPantoPos.set(xPos_mm, yPos_mm);
            
            xPos_mm       = -pos[0];
            yPos_mm       = pos[1];
            float P2x     = pos[2];
            float P2y     = pos[3];
            float P4x     = pos[4];
            float P4y     = pos[5];
            float P24norm = pos[6];
            float P2Hnorm = pos[7];
            float P3Hnorm = pos[8];
            
            // Calculate velocity for haptic effects
            PVector newPantoVel = new PVector(xPos_mm - previousPantoPos.x, yPos_mm - previousPantoPos.y);
            newPantoVel.mult(60); // Convert to mm/s (assuming 60 FPS)
            pantoVelocity.lerp(newPantoVel, 0.5); // Smooth velocity
            
            // Compute forces in N the vector is 
            float[] forces = getForces(xPos_mm, yPos_mm);
            
            float[] torques = getTorques(-forces[0], forces[1], P2x, P2y, P4x, P4y, P24norm, P2Hnorm, P3Hnorm, theta1Rad, theta5Rad);            
            float t1 = torques[0];
            float t2 = torques[1];

            //t1 = 0;
            //t2 = 0;
            
            // Send to Arduino
            String packet = "<"
                          + nf(t1, 1, 3)
                          + ","
                          + nf(t2, 1, 3)
                          + ">\n";
            myPort.write(packet);

            if (DEBUG_MODE)
            {
               println("Sent forces: " + packet.trim());
            }
         }
         catch (NumberFormatException e)
         {
            // ignore
         }
      }
   }
   // Echo from Arduino with parsed forces
   else if (in.startsWith("<") && in.endsWith(">"))
   {
      if (DEBUG_MODE)
      {
         println("echo forces: " + in);
      }
   }
}

/*=----------------------------DYNAMICS--------------------------------------*/

// This is a helper function for serialEvent and returns the computed motor forces in Newtons based on cursor position.
// In processing frame of reference (x positive to the right);
float[] getForces(float xPos_mm, float yPos_mm)
{ 
  PVector force = new PVector(0.0, 0.0);
  
  PVector wallForce = getWallForces();
  
  force.add(wallForce);

  // append force due to asteroid - also during death states!
  if (gameRunning || deathAnimation)
  {
    PVector gameForces = calculateGameForces();
    force.add(gameForces);
  }
  // append force due to black hole
  // append force due to inertia

  return new float[] {force.x, force.y};
}

// TODO fix this function - also try to get rid of duplicate code if possible
// This is a helper function for serialEvent and returns the computed motor torques in N-mm based on the force
// output (N) and the angles of the motors (rad). It uses the jacobian to get the torques from the force.
float[] getTorques(float Fx_N, float Fy_N, float P2x, float P2y, float P4x, float P4y, float P24norm, float P2Hnorm, float P3Hnorm, float theta1Rad, float theta5Rad)
{
  // Calculate the jacobian elements
  float d2x_dtheta1 = -a1 * sin(theta1Rad);
  float d2y_dtheta1 =  a1 * cos(theta1Rad);
  float d4x_dtheta5 = -a4 * sin(theta5Rad);
  float d4y_dtheta5 =  a4 * cos(theta5Rad);
  
  float d4y_dtheta1 = 0.0f;
  float d4x_dtheta1 = 0.0f;
  float d2x_dtheta5 = 0.0f;
  float d2y_dtheta5 = 0.0f;
  
  // Calculate the magnitude derivatives in mm.
  float dd_dtheta1 = ((P4x - P2x) * (d4x_dtheta1 - d2x_dtheta1) + (P4y - P2y) * (d4y_dtheta1 - d2y_dtheta1)) / P24norm;
  float dd_dtheta5 = ((P4x - P2x) * (d4x_dtheta5 - d2x_dtheta5) + (P4y - P2y) * (d4y_dtheta5 - d2y_dtheta5)) / P24norm;

  float db_dtheta1 = dd_dtheta1 - dd_dtheta1 * (a2*a2 - a3*a3 + P24norm*P24norm) / (2 * P24norm * P24norm);
  float db_dtheta5 = dd_dtheta5 - dd_dtheta5 * (a2*a2 - a3*a3 + P24norm*P24norm) / (2 * P24norm * P24norm);

  float dh_dtheta1 = - P2Hnorm * db_dtheta1 / P3Hnorm;
  float dh_dtheta5 = - P2Hnorm * db_dtheta5 / P3Hnorm;
  
  // Calculate the coordinate derivatives for point H. dHx refers to the x coord of point H
  float dHy_dtheta1 = d2y_dtheta1 + (db_dtheta1*P24norm - dd_dtheta1*P2Hnorm) * (P4y - P2y) / (P24norm*P24norm) + (P2Hnorm/P24norm) * (d4y_dtheta1 - d2y_dtheta1);
  float dHy_dtheta5 = d2y_dtheta5 + (db_dtheta5*P24norm - dd_dtheta5*P2Hnorm) * (P4y - P2y) / (P24norm*P24norm) + (P2Hnorm/P24norm) * (d4y_dtheta5 - d2y_dtheta5);
  
  float dHx_dtheta1 = d2x_dtheta1 + (db_dtheta1*P24norm - dd_dtheta1*P2Hnorm) * (P4x - P2x) / (P24norm*P24norm) + (P2Hnorm/P24norm) * (d4x_dtheta1 - d2x_dtheta1);
  float dHx_dtheta5 = d2x_dtheta5 + (db_dtheta5*P24norm - dd_dtheta5*P2Hnorm) * (P4x - P2x) / (P24norm*P24norm) + (P2Hnorm/P24norm) * (d4x_dtheta5 - d2x_dtheta5);
  
  // Compute the Jacobian entries for point P3
  float dP3y_dtheta1 = dHy_dtheta1 - (P3Hnorm/P24norm) * (d4x_dtheta1 - d2x_dtheta1) - ((dh_dtheta1*P24norm - dd_dtheta1*P3Hnorm) / (P24norm*P24norm)) * (P4x - P2x);
  float dP3y_dtheta5 = dHy_dtheta5 - (P3Hnorm/P24norm) * (d4x_dtheta5 - d2x_dtheta5) - ((dh_dtheta5*P24norm - dd_dtheta5*P3Hnorm) / (P24norm*P24norm)) * (P4x - P2x);
  
  float dP3x_dtheta1 = dHx_dtheta1 + (P3Hnorm/P24norm) * (d4y_dtheta1 - d2y_dtheta1) + ((dh_dtheta1*P24norm - dd_dtheta1*P3Hnorm) / (P24norm*P24norm)) * (P4y - P2y);
  float dP3x_dtheta5 = dHx_dtheta5 + (P3Hnorm/P24norm) * (d4y_dtheta5 - d2y_dtheta5) + ((dh_dtheta5*P24norm - dd_dtheta5*P3Hnorm) / (P24norm*P24norm)) * (P4y - P2y);

  // Calculate the torque to output.
  float t1 = -(dP3x_dtheta1 * Fx_N + dP3y_dtheta1 * Fy_N);
  float t2 = (dP3x_dtheta5 * Fx_N + dP3y_dtheta5 * Fy_N);
  
  if (DEBUG_MODE)
  {
    println("t1=" + nf(t1, 1, 3) + "  t2=" + nf(t2, 1, 3));
  }
  
  return new float[] { t1, t2 };
}

// TODO fix comments
// This is a helper function for serialEvent and computes the forward kinematics from theta1 and theta5 
// and returns the x and y position in real world mm.
float[] computeXYfromAngles_mm(float theta1Rad, float theta5Rad)
{
  float P2x = a1 * cos(theta1Rad);
  float P2y = a1 * sin(theta1Rad);
  float P4x = a4 * cos(theta5Rad) - PIVOT_OFFSET; // Pivot offset is 0
  float P4y = a4 * sin(theta5Rad);

  // // Vector from P2 to P4
  float P24x = P4x - P2x;
  float P24y = P4y - P2y;
  float P24norm = sqrt(P24x * P24x + P24y * P24y);
  
  float P2Hnorm = ((P24norm*P24norm) + (a2*a2) - (a3*a3)) / (2*P24norm);
  
  float PHx = P2x + P2Hnorm/P24norm * (P4x - P2x);
  float PHy = P2y + P2Hnorm/P24norm * (P4y - P2y);
  
  float P3Hnorm = sqrt( (a2*a2) - (P2Hnorm*P2Hnorm) );
  
  float P3x = PHx + P3Hnorm/P24norm * (P4y - P2y);
  float P3y = PHy - P3Hnorm/P24norm * (P4x - P2x);

  // Return (x, y) as a float array
  return new float[] { P3x, P3y, P2x, P2y, P4x, P4y, P24norm, P2Hnorm, P3Hnorm };
}

/****************************************************************************
***************************CODE INSERT START*********************************
*****************************************************************************/

// This function calculates the forces from the wall. The walls act as a spring.
PVector getWallForces()
{
  PVector force = new PVector(0, 0);
  
  // This should be its own function which checks if in the walls//////////////////////
  // Calculate the force for the x direction.
  if (xPos_mm < X_MIN_MM)
  {
    force.x = -WALL_SPRING_CONST * (xPos_mm - X_MIN_MM);
  }
  else if (xPos_mm > X_MAX_MM)
  {
    force.x = -WALL_SPRING_CONST * (xPos_mm - X_MAX_MM);
  }
  else
  {
    force.x = 0.0;
  }
  
  // Calculate the force for the y direction
  if (yPos_mm < Y_MIN_MM)
  {
    force.y = -WALL_SPRING_CONST * (yPos_mm - Y_MIN_MM);
  }
  else if (yPos_mm > Y_MAX_MM)
  {
    force.y = -WALL_SPRING_CONST * (yPos_mm - Y_MAX_MM);
  }
  else
  {
    force.y = 0.0;
  }
  return force;
}

// Game force calculation
PVector calculateGameForces() {
  PVector currentForce = new PVector(0, 0);
  
  // Apply asteroid collision force if active (works even when game is over)
  if (asteroidCollisionStartTime > 0) {
    long currentTime = millis();
    if (currentTime - asteroidCollisionStartTime < COLLISION_FORCE_DURATION_MS) {
      currentForce.add(asteroidCollisionForce);
    } else {
      // Reset collision force when duration expires
      asteroidCollisionForce.set(0, 0);
      asteroidCollisionStartTime = 0;
    }
  }
  
  // Apply black hole gravitational forces if game is running
  if (gameRunning || deathAnimation) {
    // Convert screen position to window coordinates for UFO
    float xPosWindow = (xPos_mm - X_MIN_MM) * (width / (X_MAX_MM - X_MIN_MM));
    float yPosWindow = (yPos_mm - Y_MIN_MM) * (height / (Y_MAX_MM - Y_MIN_MM));
    PVector ufoPos = new PVector(xPosWindow, yPosWindow);
    
    for (BlackHole bh : blackHoles) {
      PVector blackHoleForce = calculateBlackHoleForce(bh, ufoPos);
      currentForce.add(blackHoleForce);
    }
  }
  
  return currentForce;
}

// Black hole force calculation - inverse square law with mass scaling
PVector calculateBlackHoleForce(BlackHole bh, PVector ufoPos)
{
  // Calculate distance between UFO center and black hole center
  float distance = PVector.dist(ufoPos, bh.pos);
  
  // This is to avoid division by 0, so clamp the minimum distance.
  float minDist = max(distance, bh.size * 0.10);
  
  // Calculate the force influence on the UFO from the black hole
  float forceMag = virtualMass * BLACK_HOLE_GRAVITY_CONST / (minDist * minDist); // TODO add the spacecraft mass later
  
  PVector forceDir = PVector.sub(bh.pos, ufoPos);
  forceDir.mult(forceMag);
  
  float forceX_mm = forceDir.x * (X_MAX_MM - X_MIN_MM) / width;
  float forceY_mm = forceDir.y * (Y_MAX_MM - Y_MIN_MM) / height;
    
  return new PVector(forceX_mm, forceY_mm);
}

// Game logic updates
void updateGameLogic() {
  if (gameRunning) {
    // Update invincibility
    if (invincible) {
      invincibleTimer -= 1.0/60.0;
      if (invincibleTimer <= 0) {
        invincible = false;
      }
    }
    
    // Update virtual mass based on cargo
    virtualMass = 1.0 + cargoCollected * CARGO_MASS_INCREMENT;
    
    // Create smoke particles if damaged (more frequent at 1 heart)
    int smokeFrequency = (damageLevel >= 4) ? 3 : (10 - damageLevel * 2); // Heavy smoke at 1 heart (damageLevel 4)
    if (damageLevel > 0 && frameCount % smokeFrequency == 0) {
      createSmokeParticle();
    }
  }
  
  // Update UFO spinning animation
  ufoSpinAngle += 0.05;
  if (ufoSpinAngle > TWO_PI) {
    ufoSpinAngle -= TWO_PI;
  }
  
  frameSpinAngle += 0.08;
  if (frameSpinAngle > TWO_PI) {
    frameSpinAngle -= TWO_PI;
  }
  
  // Update wall glow pulse
  wallGlowPulse += 0.03;
  if (wallGlowPulse > TWO_PI) {
    wallGlowPulse -= TWO_PI;
  }
}

void updateSpeedScaling() {
  // Check for speed increases at 3x, 6x, 9x mass increments
  int currentIncrement = (int)(virtualMass / 3.0);
  if (currentIncrement > lastSpeedIncrement) {
    baseAsteroidSpeed *= 2.0;
    baseCargoSpeed *= 2.0;
    lastSpeedIncrement = currentIncrement;
    
    // Visual feedback for speed increase
    float xPosWindow = (xPos_mm - X_MIN_MM) * (width / (X_MAX_MM - X_MIN_MM));
    float yPosWindow = (yPos_mm - Y_MIN_MM) * (height / (Y_MAX_MM - Y_MIN_MM));
    createExplosion(new PVector(xPosWindow, yPosWindow), color(255, 255, 0));
  }
}

void updateScoreBasedScaling() {
  // Check for score milestones every 100 points
  int currentMilestone = score / 100;
  if (currentMilestone > lastScoreMilestone) {
    lastScoreMilestone = currentMilestone;
    
    // Increase asteroid spawn rate by 5% (multiply by 1.05)
    asteroidSpawnMultiplier *= 1.05;
    
    // Increase speed by 5% (multiply by 1.05)
    speedMultiplier *= 1.05;
    
    // Visual feedback for milestone reached
    float xPosWindow = (xPos_mm - X_MIN_MM) * (width / (X_MAX_MM - X_MIN_MM));
    float yPosWindow = (yPos_mm - Y_MIN_MM) * (height / (Y_MAX_MM - Y_MIN_MM));
    createExplosion(new PVector(xPosWindow, yPosWindow), color(100, 255, 255));
  }
}

void checkCollisions(float xPosWindow, float yPosWindow) {
  if (!gameRunning) return; // No collisions on title screen
  
  PVector playerPos = new PVector(xPosWindow, yPosWindow);
  
  // Check asteroid collisions (updated for 75% larger objects)
  if (!invincible) {
    for (int j = asteroids.size() - 1; j >= 0; j--) {
      Asteroid ast = asteroids.get(j);
      if (PVector.dist(playerPos, ast.pos) < 70) { // Increased collision radius for larger objects
        
        // Calculate collision force vector (from asteroid center to UFO center)
        PVector forceDirection = PVector.sub(playerPos, ast.pos);
        forceDirection.normalize();
        
        // Scale force based on asteroid size (52-105 pixels maps to 3-8 N)
        float forceMagnitude = map(ast.size, 52, 105, 3.0, 8.0);
        forceDirection.mult(forceMagnitude);
        
        // Convert screen coordinates to mm space for force application
        float forceX_mm = forceDirection.x * (X_MAX_MM - X_MIN_MM) / width;
        float forceY_mm = forceDirection.y * (Y_MAX_MM - Y_MIN_MM) / height;
        
        // Set collision force and start timer using millis()
        asteroidCollisionForce.set(forceX_mm, forceY_mm);
        asteroidCollisionStartTime = millis();
        
        breakAsteroid(ast);
        asteroids.remove(j);
        
        lives--;
        damageLevel = maxLives - lives;
        createExplosion(playerPos, color(255, 100, 100));
        if (lives <= 0) {
          // Start death animation immediately with explosion
          gameRunning = false;
          deathAnimation = true;
          deathAnimationTimer = 0;
          blackHoleDeath = false; // This is an asteroid death, not black hole
          
          // Create massive UFO explosion
          createUFOExplosion(playerPos);
        } else {
          invincible = true;
          invincibleTimer = 2.0;
        }
        break;
      }
    }
  }
  
  // Check cargo collection (updated for 75% larger objects)
  for (int i = cargoItems.size() - 1; i >= 0; i--) {
    Cargo cargo = cargoItems.get(i);
    if (!cargo.beingSuckedIn && PVector.dist(playerPos, cargo.pos) < 75) { // Can't collect cargo being sucked in
      cargoCollected++;
      score += 50;
      createExplosion(cargo.pos, color(255, 255, 100));
      cargoItems.remove(i);
    }
  }
  
  // Check black hole collision (updated for 75% larger objects)
  for (BlackHole bh : blackHoles) {
    if (PVector.dist(playerPos, bh.pos) < bh.eventHorizon) {
      // Start black hole death animation
      gameRunning = false;
      deathAnimation = true;
      deathAnimationTimer = 0;
      blackHoleDeath = true;
      blackHoleDeathPos.set(bh.pos);
      ufoDeathPos.set(playerPos);
      ufoDeathSize = 200; // Updated for 200px UFO (was 105, now 200)
      break;
    }
  }
}

void createSmokeParticle() {
  float xPosWindow = (xPos_mm - X_MIN_MM) * (width / (X_MAX_MM - X_MIN_MM));
  float yPosWindow = (yPos_mm - Y_MIN_MM) * (height / (Y_MAX_MM - Y_MIN_MM));
  
  PVector smokePos = new PVector(xPosWindow, yPosWindow);
  smokePos.add(random(-35, 35), random(-35, 35)); // 75% larger spread
  PVector smokeVel = new PVector(random(-0.5, 0.5), random(-2, -0.5));
  smokeParticles.add(new SmokeParticle(smokePos, smokeVel));
  
  // At 1 heart (damageLevel 4), add fire particles and mini flames
  if (damageLevel >= 4) {
    // Main fire particles
    PVector firePos = new PVector(xPosWindow, yPosWindow);
    firePos.add(random(-20, 20), random(-20, 20));
    PVector fireVel = PVector.random2D();
    fireVel.mult(random(1, 3));
    particles.add(new Particle(firePos, fireVel, color(255, random(100, 200), 0))); // Fire colors
    
    // Mini flames coming off the UFO
    for (int i = 0; i < 3; i++) {
      PVector flamePos = new PVector(xPosWindow, yPosWindow);
      float angle = random(TWO_PI);
      float dist = random(30, 50); // Just outside UFO radius
      flamePos.add(cos(angle) * dist, sin(angle) * dist);
      PVector flameVel = new PVector(cos(angle), sin(angle));
      flameVel.mult(random(0.5, 1.5));
      particles.add(new Particle(flamePos, flameVel, color(255, random(150, 255), random(0, 50)))); // Orange-red flames
    }
  }
}

void createExplosion(PVector pos, color col) {
  for (int i = 0; i < 10; i++) {
    PVector vel = PVector.random2D();
    vel.mult(random(2, 8));
    particles.add(new Particle(pos.copy(), vel, col));
  }
}

void createUFOExplosion(PVector pos) {
  // Create a massive explosion with multiple colors and sizes
  for (int i = 0; i < 150; i++) { // Much bigger explosion (was 50, now 150)
    PVector vel = PVector.random2D();
    vel.mult(random(3, 25)); // Faster and more spread out particles (was 3-15, now 3-25)
    
    // Mix of fire, smoke, and electrical colors
    color explosionColor;
    float colorChoice = random(1);
    if (colorChoice < 0.4) {
      explosionColor = color(255, random(100, 255), random(0, 50)); // Orange/red fire
    } else if (colorChoice < 0.7) {
      explosionColor = color(255, 255, random(100, 255)); // Yellow/white hot
    } else {
      explosionColor = color(random(100, 255), random(150, 255), 255); // Blue/purple electrical
    }
    
    particles.add(new Particle(pos.copy(), vel, explosionColor));
  }
  
  // Add many more slower, larger smoke particles
  for (int i = 0; i < 80; i++) { // More smoke particles (was 20, now 80)
    PVector vel = PVector.random2D();
    vel.mult(random(1, 8)); // Faster smoke spread (was 1-4, now 1-8)
    smokeParticles.add(new SmokeParticle(pos.copy(), vel));
  }
  
  // Add some extra debris particles for more visual impact
  for (int i = 0; i < 50; i++) {
    PVector vel = PVector.random2D();
    vel.mult(random(2, 12));
    particles.add(new Particle(pos.copy(), vel, color(random(100, 200), random(100, 200), random(100, 200))));
  }
}

void checkHighScore() {
  for (int i = 0; i < 5; i++) {
    if (score > highScores[i]) {
      enteringName = true;
      nameIndex = i;
      currentPlayerName = "";
      return;
    }
  }
}

// Key handling
void keyPressed() {
  if (showPressStart) {
    if (key == ' ') {
      showPressStart = false;
      gameRunning = true;
      
      // Keep existing asteroids and black holes from title screen
      // Add additional game objects (25% more asteroids)
      for (int i = 0; i < 4; i++) { // Increased from 3 to 4 (25% more)
        asteroids.add(new Asteroid());
      }
      
      for (int i = 0; i < 2; i++) {
        cargoItems.add(new Cargo());
      }
    } else if (key == 'l' || key == 'L') {
      showLeaderboard = true;
      showPressStart = false;
    } else if (key == CODED) {
      if (keyCode == LEFT) {
        currentAvatar = (currentAvatar - 1 + 7) % 7; // Cycle backwards through avatars
      } else if (keyCode == RIGHT) {
        currentAvatar = (currentAvatar + 1) % 7; // Cycle forwards through avatars
      }
    }
  } else if (enteringName) {
    if (key == ENTER || key == RETURN) {
      if (currentPlayerName.length() == 0) {
        currentPlayerName = "PLAYER";
      }
      
      for (int i = 4; i > nameIndex; i--) {
        highScores[i] = highScores[i-1];
        playerNames[i] = playerNames[i-1];
      }
      highScores[nameIndex] = score;
      playerNames[nameIndex] = currentPlayerName.toUpperCase();
      
      enteringName = false;
      showLeaderboard = true;
    } else if (key == BACKSPACE) {
      if (currentPlayerName.length() > 0) {
        currentPlayerName = currentPlayerName.substring(0, currentPlayerName.length() - 1);
      }
    } else if (key >= 'a' && key <= 'z' || key >= 'A' && key <= 'Z' || key >= '0' && key <= '9') {
      if (currentPlayerName.length() < 10) {
        currentPlayerName += key;
      }
    }
  } else if (showLeaderboard) {
    if (key == ESC) {
      showLeaderboard = false;
      showPressStart = true;
      key = 0;
    } else if (key == ' ') {
      restartGame();
    }
  } else {
    if (key == ' ') {
      restartGame();
    } else if (key == 'l' || key == 'L') {
      showLeaderboard = true;
    }
  }
}

void restartGame() {
  score = 0;
  lives = 5; // Reset to 5 hearts
  cargoCollected = 0;
  damageLevel = 0;
  gameRunning = false;
  showPressStart = true;
  showLeaderboard = false;
  enteringName = false;
  deathAnimation = false; // Reset death animation
  deathAnimationTimer = 0;
  blackHoleDeath = false; // Reset black hole death flag
  gameTime = 0;
  gameSpeedMultiplier = 1.0;
  ufoSpinAngle = 0;
  frameSpinAngle = 0;
  virtualMass = 1.0;
  invincible = false;
  invincibleTimer = 0;
  lastSpeedIncrement = 0;
  baseAsteroidSpeed = 1.0;
  baseCargoSpeed = 3.0;
  
  // Reset score-based scaling
  lastScoreMilestone = 0;
  asteroidSpawnMultiplier = 1.0;
  speedMultiplier = 1.0;
  
  // Avatar selection persists across games (don't reset currentAvatar)
  
  // Reset collision force tracking
  asteroidCollisionForce.set(0, 0);
  asteroidCollisionStartTime = 0;
  
  asteroids.clear();
  cargoItems.clear();
  particles.clear();
  blackHoles.clear();
  asteroidFragments.clear();
  raceLights.clear();
  smokeParticles.clear();
  
  stars.clear();
  for (int i = 0; i < (width * height) / 8000; i++) {
    stars.add(new Star());
  }
}

 
/****************************************************************************
***************************CODE INSERT END***********************************
*****************************************************************************/


/*=----------------------------OBJECTS--------------------------------------*/

/****************************************************************************
***************************CODE INSERT START*********************************
*****************************************************************************/

class SmokeParticle {
  PVector pos, vel;
  float life, maxLife;
  float size;
  color smokeColor;
  
  SmokeParticle(PVector p, PVector v) {
    pos = p.copy();
    vel = v.copy();
    maxLife = life = random(40, 80);
    size = random(4.5, 12); // 50% larger (was 3-8, now 4.5-12)
    smokeColor = color(80, 80, 80);
  }
  
  void update() {
    pos.add(vel);
    vel.mult(0.95);
    vel.add(0, -0.1);
    life--;
    size += 0.15; // Slightly faster growth for larger particles
  }
  
  boolean isDead() {
    return life <= 0;
  }
  
  void draw() {
    float alpha = map(life, 0, maxLife, 0, 75); // 50% less transparent (was 150, now 75)
    fill(red(smokeColor), green(smokeColor), blue(smokeColor), alpha);
    noStroke();
    ellipse(pos.x, pos.y, size, size);
  }
}

class RaceLight {
  PVector pos;
  float life, maxLife;
  color lightColor;
  float size;
  
  RaceLight(PVector p) {
    pos = p.copy();
    maxLife = life = random(20, 40);
    lightColor = color(255, 150 + random(50), 255, 200);
    size = random(2, 8);
  }
  
  void update() {
    life--;
  }
  
  boolean isDead() {
    return life <= 0;
  }
  
  void draw() {
    float alpha = map(life, 0, maxLife, 0, 255);
    float currentSize = size * (life / maxLife);
    
    fill(red(lightColor), green(lightColor), blue(lightColor), alpha * 0.3);
    noStroke();
    ellipse(pos.x, pos.y, currentSize * 3, currentSize * 3);
    
    fill(red(lightColor), green(lightColor), blue(lightColor), alpha);
    ellipse(pos.x, pos.y, currentSize, currentSize);
    
    fill(255, 255, 255, alpha);
    ellipse(pos.x, pos.y, currentSize * 0.3, currentSize * 0.3);
  }
}

class Star {
  PVector pos, vel;
  float brightness, size;
  float twinkleSpeed, twinklePhase, baseBrightness;
  
  Star() {
    pos = new PVector(random(width), random(height));
    vel = new PVector(0, random(0.225, 0.975)); // 1.5x faster (was 0.15-0.65, now 0.225-0.975)
    baseBrightness = random(50, 255);
    brightness = baseBrightness;
    size = random(1.0, 4.0); // Twice as large (was 0.5-2.0, now 1.0-4.0)
    twinkleSpeed = random(0.01, 0.04); // Slower twinkling (was 0.05-0.15, now 0.01-0.04)
    twinklePhase = random(TWO_PI); // Random starting phase
  }
  
  void update() {
    pos.add(vel);
    
    // Update twinkling
    twinklePhase += twinkleSpeed;
    if (twinklePhase > TWO_PI) {
      twinklePhase -= TWO_PI;
    }
    
    // Calculate twinkling brightness with more dramatic variation
    float twinkleFactor = 0.3 + 0.7 * sin(twinklePhase); // Varies between 0.3 and 1.0 (much more dramatic)
    
    // Add occasional bright flickers
    if (random(1) < 0.05) { // 5% chance per frame
      twinkleFactor *= 1.5; // Bright flicker
    }
    
    brightness = baseBrightness * constrain(twinkleFactor, 0.2, 1.2);
  }
  
  boolean isOffScreen() {
    return pos.y > height + 10;
  }
  
  void reset() {
    pos.y = random(-50, -10);
    pos.x = random(width);
    vel.y = random(0.225, 0.975); // 1.5x faster (was 0.15-0.65, now 0.225-0.975)
    baseBrightness = random(50, 255);
    brightness = baseBrightness;
    size = random(1.0, 4.0); // Twice as large (was 0.5-2.0, now 1.0-4.0)
    twinkleSpeed = random(0.01, 0.04); // Slower twinkling (was 0.05-0.15, now 0.01-0.04)
    twinklePhase = random(TWO_PI); // Random starting phase
  }
  
  void draw() {
    fill(255, brightness);
    noStroke();
    ellipse(pos.x, pos.y, size, size);
  }
}

class Asteroid {
  PVector pos, vel;
  float size, rotation, rotSpeed;
  ArrayList<PVector> vertices;
  ArrayList<Crater> craters;
  
  // Black hole interaction variables
  boolean beingPulled = false;
  float pullTimer = 0;
  final float PULL_DURATION = 1.0; // 1 second
  boolean beingSuckedIn = false;
  PVector suckTarget = new PVector(0, 0);
  PVector originalVel = new PVector(0, 0);
  float suckTimer = 0;
  final float SUCK_DURATION = 1.0; // Duration for sucking animation
  float originalSize;
  
  Asteroid() {
    pos = new PVector(random(width), random(-100, -50));
    vel = new PVector(random(-1, 1), random(1, 3) * baseAsteroidSpeed * speedMultiplier);
    originalVel = vel.copy();
    size = random(52, 105); // 75% larger (was 30-60, now 52-105)
    originalSize = size;
    rotation = 0;
    rotSpeed = random(-0.05, 0.05);
    
    vertices = new ArrayList<PVector>();
    int numVertices = int(random(8, 16));
    for (int i = 0; i < numVertices; i++) {
      float angle = TWO_PI * i / numVertices;
      float radius = size/2 * random(0.7, 1.3);
      vertices.add(new PVector(cos(angle) * radius, sin(angle) * radius));
    }
    
    craters = new ArrayList<Crater>();
    int numCraters = int(random(8, 15)); // More craters (was 3-8, now 8-15)
    for (int i = 0; i < numCraters; i++) {
      float angle = random(TWO_PI);
      float dist = random(size * 0.05, size * 0.45); // Spread throughout asteroid (was 0.1-0.3, now 0.05-0.45)
      float x = cos(angle) * dist;
      float y = sin(angle) * dist;
      float craterSize = random(size * 0.05, size * 0.15);
      craters.add(new Crater(x, y, craterSize));
    }
  }
  
  void update() {
    if (beingSuckedIn) {
      // Update suck timer
      suckTimer += 1.0/60.0;
      float suckProgress = constrain(suckTimer / SUCK_DURATION, 0, 1);
      
      // Lerp position toward black hole center (like UFO)
      pos.lerp(suckTarget, suckProgress * 0.1);
      
      // Shrink size based on progress (like UFO)
      size = originalSize * (1.0 - suckProgress);
      
      // Keep normal rotation speed (no spinning acceleration)
      rotation += rotSpeed;
      
    } else {
      PVector scaledVel = PVector.mult(vel, gameSpeedMultiplier);
      pos.add(scaledVel);
      rotation += rotSpeed;
      
      // Update pull timer
      if (beingPulled) {
        pullTimer += 1.0/60.0; // Add frame time
        if (pullTimer >= PULL_DURATION) {
          beingPulled = false;
          pullTimer = 0;
          // Keep current velocity as tangent trajectory (don't restore original)
        }
      }
    }
  }
  
  void applyBlackHolePull(PVector blackHolePos, float pullStrength) {
    if (!beingPulled && !beingSuckedIn) {
      beingPulled = true;
      pullTimer = 0;
      originalVel = vel.copy(); // Store original velocity
    }
    
    if (beingPulled) {
      // Apply continuous gravitational force toward black hole (creates curved trajectory)
      PVector pullForce = PVector.sub(blackHolePos, pos);
      float distance = pullForce.mag();
      pullForce.normalize();
      
      // Distance-based scaling for stronger curves when closer
      float distanceScale = 1.0 + (300.0 / max(distance, 50)); // Stronger pull when closer
      pullForce.mult(pullStrength * 0.12 * distanceScale); // Much stronger curves with distance scaling
      vel.add(pullForce); // Direct velocity modification for proper physics
    }
  }
  
  void startSuckingIn(PVector blackHoleCenter) {
    beingSuckedIn = true;
    suckTarget = blackHoleCenter.copy();
    beingPulled = false; // Stop normal pulling
    suckTimer = 0;
  }
  
  boolean shouldRemove() {
    return beingSuckedIn && suckTimer >= SUCK_DURATION; // Remove when animation complete
  }
  
  boolean isOffScreen() {
    return pos.y > height + size && !beingSuckedIn;
  }
  
  void draw() {
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(rotation);
    
    // Calculate scale factor for shrinking animation
    float scaleFactor = size / originalSize;
    
    fill(20, 15, 10);
    noStroke();
    beginShape();
    for (PVector vertex : vertices) {
      vertex(vertex.x * scaleFactor + 2, vertex.y * scaleFactor + 3);
    }
    endShape(CLOSE);
    
    fill(90, 80, 70);
    stroke(60, 50, 40);
    strokeWeight(2);
    beginShape();
    for (PVector vertex : vertices) {
      vertex(vertex.x * scaleFactor, vertex.y * scaleFactor);
    }
    endShape(CLOSE);
    
    fill(110, 100, 90);
    noStroke();
    beginShape();
    for (int i = 0; i < vertices.size(); i++) {
      PVector vertex = vertices.get(i);
      vertex(vertex.x * scaleFactor - 1, vertex.y * scaleFactor - 1.5);
    }
    endShape(CLOSE);
    
    for (Crater crater : craters) {
      crater.drawScaled(scaleFactor);
    }
    
    stroke(110, 100, 90);
    strokeWeight(1);
    for (int i = 0; i < 4; i++) {
      float lineX = random(-size/3, size/3);
      float lineY = -size/2 - random(10, 20);
      line(lineX, lineY, lineX, lineY + random(5, 15));
    }
    
    popMatrix();
  }
}

class Crater {
  float x, y, size;
  
  Crater(float x, float y, float size) {
    this.x = x;
    this.y = y;
    this.size = size;
  }
  
  void draw() {
    fill(25, 20, 15);
    noStroke();
    ellipse(x + size * 0.2, y + size * 0.3, size, size * 0.8);
    
    fill(35, 30, 25);
    ellipse(x, y, size, size * 0.8);
    
    fill(50, 45, 40);
    ellipse(x - size * 0.1, y - size * 0.1, size * 0.6, size * 0.5);
  }
  
  void drawScaled(float scaleFactor) {
    float scaledSize = size * scaleFactor;
    float scaledX = x * scaleFactor;
    float scaledY = y * scaleFactor;
    
    fill(25, 20, 15);
    noStroke();
    ellipse(scaledX + scaledSize * 0.2, scaledY + scaledSize * 0.3, scaledSize, scaledSize * 0.8);
    
    fill(35, 30, 25);
    ellipse(scaledX, scaledY, scaledSize, scaledSize * 0.8);
    
    fill(50, 45, 40);
    ellipse(scaledX - scaledSize * 0.1, scaledY - scaledSize * 0.1, scaledSize * 0.6, scaledSize * 0.5);
  }
}

class Cargo {
  PVector pos, vel;
  float size, glow, glowDirection;
  boolean collected = false;
  
  // Black hole interaction variables
  boolean beingPulled = false;
  float pullTimer = 0;
  final float PULL_DURATION = 1.0; // 1 second
  boolean beingSuckedIn = false;
  PVector suckTarget = new PVector(0, 0);
  PVector originalVel = new PVector(0, 0);
  float suckTimer = 0;
  final float SUCK_DURATION = 1.0; // Duration for sucking animation
  float originalSize;
  
  Cargo() {
    pos = new PVector(random(50, width-50), random(-100, -50));
    vel = new PVector(random(-0.5, 0.5), random(0.5, 1.5) * baseCargoSpeed * speedMultiplier);
    originalVel = vel.copy();
    size = 44; // 75% larger (was 25, now 44)
    originalSize = size;
    glow = 0;
    glowDirection = 1;
  }
  
  void update() {
    if (beingSuckedIn) {
      // Update suck timer
      suckTimer += 1.0/60.0;
      float suckProgress = constrain(suckTimer / SUCK_DURATION, 0, 1);
      
      // Lerp position toward black hole center (like UFO)
      pos.lerp(suckTarget, suckProgress * 0.1);
      
      // Shrink size based on progress (like UFO)
      size = originalSize * (1.0 - suckProgress);
      
    } else {
      pos.add(vel);
      
      glow += glowDirection * 0.03;
      if (glow > 1 || glow < 0) {
        glowDirection *= -1;
        glow = constrain(glow, 0, 1);
      }
      
      // Update pull timer
      if (beingPulled) {
        pullTimer += 1.0/60.0; // Add frame time
        if (pullTimer >= PULL_DURATION) {
          beingPulled = false;
          pullTimer = 0;
          // Keep current velocity as tangent trajectory (don't restore original)
        }
      }
    }
  }
  
  void applyBlackHolePull(PVector blackHolePos, float pullStrength) {
    if (!beingPulled && !beingSuckedIn) {
      beingPulled = true;
      pullTimer = 0;
      originalVel = vel.copy(); // Store original velocity
    }
    
    if (beingPulled) {
      // Apply continuous gravitational force toward black hole (creates curved trajectory)
      PVector pullForce = PVector.sub(blackHolePos, pos);
      float distance = pullForce.mag();
      pullForce.normalize();
      
      // Distance-based scaling for stronger curves when closer
      float distanceScale = 1.0 + (300.0 / max(distance, 50)); // Stronger pull when closer
      pullForce.mult(pullStrength * 0.12 * distanceScale); // Much stronger curves with distance scaling
      vel.add(pullForce); // Direct velocity modification for proper physics
    }
  }
  
  void startSuckingIn(PVector blackHoleCenter) {
    beingSuckedIn = true;
    suckTarget = blackHoleCenter.copy();
    beingPulled = false; // Stop normal pulling
    suckTimer = 0;
  }
  
  boolean shouldRemove() {
    return beingSuckedIn && suckTimer >= SUCK_DURATION; // Remove when animation complete
  }
  
  boolean isOffScreen() {
    return pos.y > height + size && !beingSuckedIn;
  }
  
  void draw() {
    pushMatrix();
    translate(pos.x, pos.y);
    
    fill(255, 200, 100, 30 + glow * 70);
    noStroke();
    ellipse(0, 0, size * 3.5, size * 3.5);
    
    fill(255, 220, 120, 60 + glow * 120);
    ellipse(0, 0, size * 2.5, size * 2.5);
    
    fill(255, 240, 150, 80 + glow * 150);
    ellipse(0, 0, size * 1.8, size * 1.8);
    
    fill(180, 120, 60);
    stroke(120, 80, 40);
    strokeWeight(2);
    rectMode(CENTER);
    rect(0, 0, size, size);
    
    fill(255, 240, 150);
    textAlign(CENTER);
    textSize(10);
    text("CARGO", 0, 3);
    
    stroke(255, 240, 150, 150 + glow * 105);
    strokeWeight(2);
    for (int i = 0; i < 8; i++) {
      float angle = TWO_PI * i / 8;
      float x1 = cos(angle) * size * 0.8;
      float y1 = sin(angle) * size * 0.8;
      float x2 = cos(angle) * (size * 0.8 + 15 + glow * 10);
      float y2 = sin(angle) * (size * 0.8 + 15 + glow * 10);
      line(x1, y1, x2, y2);
    }
    
    fill(255, 255, 200, 100 + glow * 155);
    noStroke();
    for (int i = 0; i < 6; i++) {
      float sparkleAngle = TWO_PI * i / 6 + frameCount * 0.05;
      float sparkleX = cos(sparkleAngle) * (size * 0.6 + glow * 8);
      float sparkleY = sin(sparkleAngle) * (size * 0.6 + glow * 8);
      ellipse(sparkleX, sparkleY, 3 + glow * 2, 3 + glow * 2);
    }
    
    popMatrix();
  }
}

class BlackHole {
  PVector pos;
  float size, eventHorizon;
  float rotation;
  float lifeTime, maxLifeTime;
  
  BlackHole() {
    // Calculate UFO position in screen coordinates
    float ufoX = (xPos_mm - X_MIN_MM) * (width / (X_MAX_MM - X_MIN_MM));
    float ufoY = (yPos_mm - Y_MIN_MM) * (height / (Y_MAX_MM - Y_MIN_MM));
    PVector ufoPos = new PVector(ufoX, ufoY);
    
    // Find a spawn position at least 450 pixels away from UFO
    do {
      pos = new PVector(random(100, width-100), random(50, height-200));
    } while (PVector.dist(pos, ufoPos) < 450); // Increased no-go radius (was 200, now 450)
    
    size = 149; // 75% larger (was 85, now 149)
    eventHorizon = size * 0.4;
    rotation = 0;
    maxLifeTime = lifeTime = random(600, 1200);
  }
  
  void update() {
    rotation += 0.02;
    lifeTime--;
  }
  
  boolean shouldRemove() {
    return lifeTime <= 0;
  }
  
  boolean isOffScreen() {
    return pos.x < -size || pos.x > width + size || pos.y < -size || pos.y > height + size;
  }
  
  void draw() {
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(rotation);
    
    noFill();
    for (int i = 1; i <= 6; i++) {
      stroke(100, 50, 200, 60 - i * 8);
      strokeWeight(2);
      ellipse(0, 0, size + i * 30, size + i * 30);
    }
    
    fill(20, 10, 40);
    stroke(80, 40, 160);
    strokeWeight(3);
    ellipse(0, 0, size, size);
    
    noFill();
    stroke(150, 100, 255);
    strokeWeight(2);
    beginShape();
    for (float a = 0; a < TWO_PI * 3; a += 0.1) {
      float r = (size/2 - 5) * (1 - a / (TWO_PI * 3));
      float x = cos(a + rotation * 2) * r;
      float y = sin(a + rotation * 2) * r;
      vertex(x, y);
    }
    endShape();
    
    fill(0, 0, 0);
    stroke(150, 100, 255);
    strokeWeight(2);
    ellipse(0, 0, size * 0.25, size * 0.25);
    
    stroke(200, 150, 255);
    strokeWeight(2);
    for (int i = 0; i < 6; i++) {
      float angle = TWO_PI * i / 6 + rotation;
      float dist = size/2 + 25;
      float x = cos(angle) * dist;
      float y = sin(angle) * dist;
      
      pushMatrix();
      translate(x, y);
      rotate(angle + PI/2);
      line(-5, 0, 5, 0);
      line(3, -2, 5, 0);
      line(3, 2, 5, 0);
      popMatrix();
    }
    
    popMatrix();
  }
}

class AsteroidFragment {
  PVector pos, vel;
  float size, rotation, rotSpeed;
  float life, maxLife;
  color fragmentColor;
  
  AsteroidFragment(PVector p, PVector v, float s) {
    pos = p.copy();
    vel = v.copy();
    size = s;
    rotation = 0;
    rotSpeed = random(-0.1, 0.1);
    maxLife = life = random(60, 120);
    fragmentColor = color(70, 60, 50);
  }
  
  void update() {
    pos.add(vel);
    vel.mult(0.98);
    rotation += rotSpeed;
    life--;
  }
  
  boolean isDead() {
    return life <= 0;
  }
  
  void draw() {
    float alpha = map(life, 0, maxLife, 0, 255);
    
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(rotation);
    
    fill(red(fragmentColor), green(fragmentColor), blue(fragmentColor), alpha);
    stroke(40, 35, 25, alpha);
    strokeWeight(1);
    
    beginShape();
    for (int i = 0; i < 6; i++) {
      float angle = TWO_PI * i / 6;
      float radius = size * random(0.7, 1.0);
      float x = cos(angle) * radius;
      float y = sin(angle) * radius;
      vertex(x, y);
    }
    endShape(CLOSE);
    
    popMatrix();
  }
}

class Particle {
  PVector pos, vel;
  color col;
  float life, maxLife;
  float size;
  
  Particle(PVector p, PVector v, color c) {
    pos = p.copy();
    vel = v.copy();
    col = c;
    maxLife = life = random(30, 60);
    size = random(3, 9); // 50% larger (was 2-6, now 3-9)
  }
  
  void update() {
    pos.add(vel);
    vel.mult(0.98);
    life--;
  }
  
  boolean isDead() {
    return life <= 0;
  }
  
  void draw() {
    float alpha = map(life, 0, maxLife, 0, 128); // 50% less transparent (was 255, now 128)
    fill(red(col), green(col), blue(col), alpha);
    noStroke();
    ellipse(pos.x, pos.y, size * (life/maxLife), size * (life/maxLife));
  }
}

void breakAsteroid(Asteroid ast) {
  int numFragments = int(random(4, 7));
  for (int i = 0; i < numFragments; i++) {
    PVector fragPos = ast.pos.copy();
    PVector fragVel = PVector.random2D();
    fragVel.mult(random(1, 4));
    fragVel.add(ast.vel);
    
    float fragSize = ast.size * random(0.2, 0.4);
    asteroidFragments.add(new AsteroidFragment(fragPos, fragVel, fragSize));
  }
}

// Update functions for game objects
void updateAsteroids() {
  for (int i = asteroids.size() - 1; i >= 0; i--) {
    Asteroid ast = asteroids.get(i);
    
    // Check interactions with black holes
    for (BlackHole bh : blackHoles) {
      float distance = PVector.dist(ast.pos, bh.pos);
      float asteroidRadius = ast.size / 2;
      float blackHoleOuterRadius = (bh.size + 250) / 2;
      
      // Check if asteroid overlaps with event horizon (inner circle)
      if (distance + asteroidRadius > bh.eventHorizon && distance - asteroidRadius < bh.eventHorizon) {
        if (!ast.beingSuckedIn) {
          ast.startSuckingIn(bh.pos);
          createExplosion(ast.pos, color(150, 50, 200)); // Purple explosion effect
        }
      }
      // Check if asteroid is within black hole outer radius for pulling
      else if (distance <= blackHoleOuterRadius + asteroidRadius && !ast.beingSuckedIn) {
        float pullStrength = 0.3; // Reduced pull force (was 0.4, now 0.3)
        ast.applyBlackHolePull(bh.pos, pullStrength);
      }
    }
    
    ast.update();
    
    if (ast.isOffScreen() || ast.shouldRemove()) {
      asteroids.remove(i);
    }
  }
}

void updateAsteroidFragments() {
  for (int i = asteroidFragments.size() - 1; i >= 0; i--) {
    AsteroidFragment frag = asteroidFragments.get(i);
    frag.update();
    if (frag.isDead()) {
      asteroidFragments.remove(i);
    }
  }
}

void updateCargo() {
  for (int i = cargoItems.size() - 1; i >= 0; i--) {
    Cargo cargo = cargoItems.get(i);
    
    // Check interactions with black holes
    for (BlackHole bh : blackHoles) {
      float distance = PVector.dist(cargo.pos, bh.pos);
      float cargoRadius = cargo.size / 2;
      float blackHoleOuterRadius = (bh.size + 250) / 2;
      
      // Check if cargo overlaps with event horizon (inner circle)
      if (distance + cargoRadius > bh.eventHorizon && distance - cargoRadius < bh.eventHorizon) {
        if (!cargo.beingSuckedIn) {
          cargo.startSuckingIn(bh.pos);
          createExplosion(cargo.pos, color(255, 255, 100)); // Yellow explosion effect
        }
      }
      // Check if cargo is within black hole outer radius for pulling
      else if (distance <= blackHoleOuterRadius + cargoRadius && !cargo.beingSuckedIn) {
        float pullStrength = 0.3; // Reduced pull force (was 0.4, now 0.3)
        cargo.applyBlackHolePull(bh.pos, pullStrength);
      }
    }
    
    cargo.update();
    
    if (cargo.isOffScreen() || cargo.shouldRemove()) {
      cargoItems.remove(i);
    }
  }
}

void updateBlackHoles() {
  for (int i = blackHoles.size() - 1; i >= 0; i--) {
    BlackHole bh = blackHoles.get(i);
    bh.update();
    
    if (bh.isOffScreen() || bh.shouldRemove()) {
      blackHoles.remove(i);
    }
  }
}

void updateParticles() {
  for (int i = particles.size() - 1; i >= 0; i--) {
    Particle p = particles.get(i);
    p.update();
    if (p.isDead()) {
      particles.remove(i);
    }
  }
}

void updateSmokeParticles() {
  for (int i = smokeParticles.size() - 1; i >= 0; i--) {
    SmokeParticle smoke = smokeParticles.get(i);
    smoke.update();
    if (smoke.isDead()) {
      smokeParticles.remove(i);
    }
  }
}

void updateRaceLights() {
  for (int i = raceLights.size() - 1; i >= 0; i--) {
    RaceLight light = raceLights.get(i);
    light.update();
    if (light.isDead()) {
      raceLights.remove(i);
    }
  }
}

void updateStars() {
  for (int i = stars.size() - 1; i >= 0; i--) {
    Star star = stars.get(i);
    star.update();
    if (star.isOffScreen()) {
      star.reset();
    }
  }
}

 
/****************************************************************************
***************************CODE INSERT END***********************************
*****************************************************************************/

/*=----------------------------GRAPHICS-------------------------------------*/

/****************************************************************************
***************************CODE INSERT START*********************************
*****************************************************************************/

void drawWallForceField() {
  // Calculate pulsing intensity - ensure proper pulsing
  float intensity = 60 + sin(wallGlowPulse) * 30; // Pulses between 30-90 alpha
  
  // Left wall force field - draw as vertical lines
  for (int x = 0; x < 24; x++) {
    float alpha = intensity * (1.0 - (float)x / 24.0); // Fade out from wall
    stroke(100, 200, 255, alpha);
    strokeWeight(1);
    line(x, 0, x, height); // Draw vertical line from top to bottom
  }
  
  // Right wall force field - draw as vertical lines  
  for (int x = 0; x < 24; x++) {
    float alpha = intensity * (1.0 - (float)x / 24.0); // Fade out from wall
    stroke(100, 200, 255, alpha);
    strokeWeight(1);
    line(width - 1 - x, 0, width - 1 - x, height); // Draw vertical line from top to bottom
  }
  
  // Add some electrical sparks along the walls
  if (frameCount % 30 == 0) { // Every half second
    // Left wall sparks
    for (int i = 0; i < 2; i++) {
      float sparkY = random(height);
      fill(200, 240, 255, 180);
      noStroke();
      ellipse(random(1, 8), sparkY, random(2, 4), random(2, 4));
    }
    
    // Right wall sparks
    for (int i = 0; i < 2; i++) {
      float sparkY = random(height);
      fill(200, 240, 255, 180);
      noStroke();
      ellipse(width - random(1, 8), sparkY, random(2, 4), random(2, 4));
    }
  }
}

void drawSpaceship(float xPosWindow, float yPosWindow) {
  pushMatrix();
  translate(xPosWindow, yPosWindow);
  
  float spaceshipSize = 120; // UFO diameter 120 pixels (was 200, now 120)
  
  // Get UFO color based on current avatar
  color[] ufoColors = {
    color(150, 100, 255), // Base UFO: purple
    color(100, 150, 255), // img1: blue
    color(80, 80, 80),    // img2: black
    color(100, 255, 150), // img3: green
    color(255, 100, 100), // img4: red
    color(255, 255, 255), // img5: white
    color(255, 150, 100), // img6: orange
    color(255, 255, 100)  // img7: yellow
  };
  
  color currentUfoColor = ufoColors[currentAvatar];
  
  // Flicker if invincible (but always show lights)
  if (!invincible || frameCount % 10 < 5) {
    // Shadow/depth layer
    fill(50, 30, 80);
    noStroke();
    ellipse(2, 4, spaceshipSize * 1.1, spaceshipSize * 0.9);
    
    // Main hull with gradient effect
    for (int i = 8; i >= 0; i--) {
      float alpha = map(i, 0, 8, 255, 100);
      float size = map(i, 0, 8, spaceshipSize, spaceshipSize * 0.6);
      fill(red(currentUfoColor) + i * 10, green(currentUfoColor) + i * 8, blue(currentUfoColor) - i * 15, alpha);
      ellipse(-i * 0.5, -i * 0.8, size, size * 0.85);
    }
    
    // Outer rim
    stroke(red(currentUfoColor) + 50, green(currentUfoColor) + 80, blue(currentUfoColor));
    strokeWeight(3);
    fill(red(currentUfoColor) - 30, green(currentUfoColor) - 20, blue(currentUfoColor) - 55);
    ellipse(0, 0, spaceshipSize, spaceshipSize * 0.85);
    
    // Inner hull detail
    fill(red(currentUfoColor) - 70, green(currentUfoColor) - 40, blue(currentUfoColor) - 95);
    stroke(red(currentUfoColor) - 10, green(currentUfoColor) + 20, blue(currentUfoColor) - 35);
    strokeWeight(2);
    ellipse(-1, -1, spaceshipSize * 0.7, spaceshipSize * 0.6);
    
    // Central cockpit dome
    for (int i = 3; i >= 0; i--) {
      float domeAlpha = map(i, 0, 3, 255, 150);
      float domeSize = map(i, 0, 3, spaceshipSize * 0.35, spaceshipSize * 0.25);
      fill(red(currentUfoColor) + 30 + i * 20, green(currentUfoColor) + 20 + i * 20, blue(currentUfoColor), domeAlpha);
      ellipse(-i * 0.3, -i * 0.5, domeSize, domeSize * 0.85);
    }
    
    // Cockpit window
    fill(220, 200, 255, 200);
    noStroke();
    ellipse(-1, -2, spaceshipSize * 0.2, spaceshipSize * 0.17);
    fill(255, 255, 255, 150);
    ellipse(-2, -3, spaceshipSize * 0.08, spaceshipSize * 0.06);
    
    // Damage effects
    if (damageLevel > 0) {
      stroke(255, 100, 100);
      strokeWeight(2);
      for (int i = 0; i < damageLevel * 3; i++) {
        float angle = random(TWO_PI);
        float startR = spaceshipSize * 0.2;
        float endR = spaceshipSize * 0.4;
        float x1 = cos(angle) * startR;
        float y1 = sin(angle) * startR;
        float x2 = cos(angle + random(-0.3, 0.3)) * endR;
        float y2 = sin(angle + random(-0.3, 0.3)) * endR;
        line(x1, y1, x2, y2);
      }
      
      fill(30, 20, 10, 150);
      noStroke();
      for (int i = 0; i < damageLevel * 2; i++) {
        float angle = random(TWO_PI);
        float dist = random(spaceshipSize * 0.2, spaceshipSize * 0.4);
        float x = cos(angle) * dist;
        float y = sin(angle) * dist;
        ellipse(x, y, random(5, 12), random(3, 8));
      }
    }
    
    // Spinning frame
    pushMatrix();
    rotate(frameSpinAngle);
    
    stroke(80, 60, 140);
    strokeWeight(3);
    for (int i = 0; i < 16; i++) {
      float angle = TWO_PI * i / 16;
      float innerRadius = spaceshipSize * 0.25;
      float outerRadius = spaceshipSize * 0.42;
      
      float x1 = cos(angle) * innerRadius + 1;
      float y1 = sin(angle) * innerRadius * 0.85 + 1;
      float x2 = cos(angle) * outerRadius + 1;
      float y2 = sin(angle) * outerRadius * 0.85 + 1;
      
      line(x1, y1, x2, y2);
    }
    
    stroke(180, 140, 255);
    strokeWeight(2);
    for (int i = 0; i < 16; i++) {
      float angle = TWO_PI * i / 16;
      float innerRadius = spaceshipSize * 0.25;
      float outerRadius = spaceshipSize * 0.42;
      
      float x1 = cos(angle) * innerRadius;
      float y1 = sin(angle) * innerRadius * 0.85;
      float x2 = cos(angle) * outerRadius;
      float y2 = sin(angle) * outerRadius * 0.85;
      
      line(x1, y1, x2, y2);
    }
    
    popMatrix();
  }
  
  // ALWAYS show spinning lights (even when flickering)
  pushMatrix();
  rotate(ufoSpinAngle);
  
  // Outer ring lights with brightness variation
  for (int i = 0; i < 12; i++) {
    float angle = TWO_PI * i / 12;
    float lightX = cos(angle) * (spaceshipSize * 0.35);
    float lightY = sin(angle) * (spaceshipSize * 0.3);
    
    float brightness = 150 + sin(frameCount * 0.1 + i * 0.5) * 105;
    
    // Bright center light
    fill(255, brightness, 255);
    noStroke();
    ellipse(lightX, lightY, 6, 6); // Scaled for larger UFO
    
    // Outer glow
    fill(255, brightness, 255, 80);
    ellipse(lightX, lightY, 12, 12); // Scaled for larger UFO
  }
  
  popMatrix();
  
  // Draw spinning avatar on top of UFO (only if avatar is selected, not base UFO)
  if (currentAvatar > 0 && avatarImages[currentAvatar - 1] != null) {
    pushMatrix();
    rotate(ufoSpinAngle); // Spin at same rate as lights
    
    // Calculate avatar size (max 100 pixels tall)
    float avatarHeight = min(100, avatarImages[currentAvatar - 1].height);
    float avatarWidth = (avatarImages[currentAvatar - 1].width * avatarHeight) / avatarImages[currentAvatar - 1].height;
    
    // Draw avatar centered on UFO
    imageMode(CENTER);
    image(avatarImages[currentAvatar - 1], 0, 0, avatarWidth, avatarHeight);
    
    popMatrix();
  }
  
  popMatrix();
}

void drawStars() {
  for (Star star : stars) {
    star.draw();
  }
}

void drawAsteroids() {
  for (Asteroid ast : asteroids) {
    ast.draw();
  }
}

void drawAsteroidFragments() {
  for (AsteroidFragment frag : asteroidFragments) {
    frag.draw();
  }
}

void drawRaceLights() {
  for (RaceLight light : raceLights) {
    light.draw();
  }
}

void drawSmokeParticles() {
  for (SmokeParticle smoke : smokeParticles) {
    smoke.draw();
  }
}

void drawCargo() {
  for (Cargo cargo : cargoItems) {
    cargo.draw();
  }
}

void drawBlackHoles() {
  for (BlackHole bh : blackHoles) {
    bh.draw();
  }
}

void drawParticles() {
  for (Particle p : particles) {
    p.draw();
  }
}

void drawShrinkingSpaceship(float xPos, float yPos, float size) {
  pushMatrix();
  translate(xPos, yPos);
  
  float scaleFactor = size / 200.0; // Scale based on 200px UFO (was 105, now 200)
  
  // Shadow/depth layer
  fill(50, 30, 80);
  noStroke();
  ellipse(2 * scaleFactor, 4 * scaleFactor, size * 1.1, size * 0.9);
  
  // Main hull with gradient effect
  for (int i = 8; i >= 0; i--) {
    float alpha = map(i, 0, 8, 255, 100);
    float hullSize = map(i, 0, 8, size, size * 0.6);
    fill(150 + i * 10, 100 + i * 8, 255 - i * 15, alpha);
    ellipse(-i * 0.5 * scaleFactor, -i * 0.8 * scaleFactor, hullSize, hullSize * 0.85);
  }
  
  // Outer rim
  stroke(200, 180, 255);
  strokeWeight(3 * scaleFactor);
  fill(120, 80, 200);
  ellipse(0, 0, size, size * 0.85);
  
  // Inner hull detail
  fill(80, 60, 160);
  stroke(140, 120, 220);
  strokeWeight(2 * scaleFactor);
  ellipse(-1 * scaleFactor, -1 * scaleFactor, size * 0.7, size * 0.6);
  
  // Central cockpit dome
  for (int i = 3; i >= 0; i--) {
    float domeAlpha = map(i, 0, 3, 255, 150);
    float domeSize = map(i, 0, 3, size * 0.35, size * 0.25);
    fill(180 + i * 20, 120 + i * 20, 255, domeAlpha);
    ellipse(-i * 0.3 * scaleFactor, -i * 0.5 * scaleFactor, domeSize, domeSize * 0.85);
  }
  
  // Cockpit window
  fill(220, 200, 255, 200);
  noStroke();
  ellipse(-1 * scaleFactor, -2 * scaleFactor, size * 0.2, size * 0.17);
  fill(255, 255, 255, 150);
  ellipse(-2 * scaleFactor, -3 * scaleFactor, size * 0.08, size * 0.06);
  
  // ALWAYS show spinning lights (scaled down)
  pushMatrix();
  rotate(ufoSpinAngle);
  
  // Outer ring lights with brightness variation
  for (int i = 0; i < 12; i++) {
    float angle = TWO_PI * i / 12;
    float lightX = cos(angle) * (size * 0.35);
    float lightY = sin(angle) * (size * 0.3);
    
    float brightness = 150 + sin(frameCount * 0.1 + i * 0.5) * 105;
    
    // Bright center light
    fill(255, brightness, 255);
    noStroke();
    ellipse(lightX, lightY, 6 * scaleFactor, 6 * scaleFactor);
    
    // Outer glow
    fill(255, brightness, 255, 80);
    ellipse(lightX, lightY, 12 * scaleFactor, 12 * scaleFactor);
  }
  
  popMatrix();
  
  // Draw spinning avatar on shrinking UFO
  if (avatarImages[currentAvatar] != null) {
    pushMatrix();
    rotate(ufoSpinAngle); // Spin at same rate as lights
    
    // Calculate avatar size (max 100 pixels tall, scaled down)
    float avatarHeight = min(100, avatarImages[currentAvatar].height) * scaleFactor;
    float avatarWidth = (avatarImages[currentAvatar].width * avatarHeight) / avatarImages[currentAvatar].height;
    
    // Draw avatar centered on shrinking UFO
    imageMode(CENTER);
    image(avatarImages[currentAvatar], 0, 0, avatarWidth, avatarHeight);
    
    popMatrix();
  }
  
  popMatrix();
}

void drawPressStartUI() {
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(90);
  text("AstroTouch", width/2, height/2 - 150);
  
  if (frameCount % 60 < 30) {
    fill(100, 255, 100);
    textSize(45);
    text("Press SPACE to Start", width/2, height/2 - 60);
  }
  
  fill(200);
  textSize(30);
  text("Move the UFO around to position it in your workspace", width/2, height/2);
  text("Use the pantograph to control the UFO", width/2, height/2 + 30);
  
  fill(150);
  textSize(26);
  text("Collect cargo, avoid asteroids and black holes", width/2, height/2 + 90);
  text("Press L for leaderboard", width/2, height/2 + 120);
  text("Use LEFT/RIGHT arrows to change avatar", width/2, height/2 + 150);
  text("After 20x mass multiplier, no more cargo will spawn!", width/2, height/2 + 180);
  
  fill(100, 255, 100);
  textSize(23);
  text("Pantograph Connected", width/2, height/2 + 210);
  text("Screen: " + width + "x" + height + " pixels (Full Screen)", width/2, height/2 + 240);
}

void drawUI() {
  fill(100, 255, 100);
  textSize(45);
  textAlign(LEFT);
  text("Score: " + nf(score, 7), 30, 60); // Support up to 9,999,999 (was nf(score, 5), now nf(score, 7))
  
  // Draw lives as hearts
  float heartX = width - 400; // Moved further left to accommodate 5 hearts (was -300, now -400)
  float heartY = 60;
  
  for (int i = 0; i < maxLives; i++) {
    if (i < lives) {
      fill(255, 50, 50);
    } else {
      fill(100, 30, 30);
    }
    drawHeart(heartX + i * 53, heartY, 30);
  }
  
  // Draw cargo counter
  fill(255, 200, 100);
  textSize(30);
  textAlign(LEFT);
  text("Cargo: " + cargoCollected, 30, height - 180);
  text("Mass: " + nf(virtualMass, 1, 1) + "x", 30, height - 150);
  text("Speed: " + nf(gameSpeedMultiplier, 1, 1) + "x", 30, height - 120);
  text("Pantograph Control Active", 30, height - 90);
  
  // Mass multiplier warning
  if (virtualMass >= 18.0) {
    fill(255, 100, 100);
    textAlign(CENTER);
    textSize(38);
    text("WARNING: No more cargo after 20x mass!", width/2, 100);
  }
  
  fill(150, 150, 255);
  textSize(23);
  textAlign(LEFT);
  text("Position: (" + nf(xPos_mm, 1, 1) + ", " + nf(yPos_mm, 1, 1) + ") mm", 30, height - 60);
}

void drawHeart(float x, float y, float size) {
  pushMatrix();
  translate(x, y);
  scale(size/20.0);
  noStroke();
  
  beginShape();
  vertex(0, 5);
  bezierVertex(-10, -5, -20, 0, -10, 15);
  vertex(0, 25);
  vertex(10, 15);
  bezierVertex(20, 0, 10, -5, 0, 5);
  endShape(CLOSE);
  
  popMatrix();
}

void drawGameOver() {
  fill(255, 50, 50);
  textAlign(CENTER);
  textSize(90);
  text("GAME OVER", width/2, height/2 - 75);
  
  fill(255);
  textSize(45);
  text("Final Score: " + score, width/2, height/2);
  text("Cargo Collected: " + cargoCollected, width/2, height/2 + 45);
  text("Final Mass Multiplier: " + nf(virtualMass, 1, 1) + "x", width/2, height/2 + 90);
  
  textSize(30);
  text("Press SPACE to restart", width/2, height/2 + 150);
  text("Press L to view leaderboard", width/2, height/2 + 180);
}

void drawNameEntry() {
  background(backgroundColor);
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(68);
  text("NEW HIGH SCORE!", width/2, height/2 - 150);
  
  fill(255);
  textSize(45);
  text("Score: " + score, width/2, height/2 - 90);
  text("Enter your name:", width/2, height/2 - 30);
  
  fill(100, 255, 100);
  textSize(60);
  String displayName = currentPlayerName;
  if (frameCount % 60 < 30) displayName += "_";
  text(displayName, width/2, height/2 + 30);
  
  fill(200);
  textSize(30);
  text("Press ENTER when done", width/2, height/2 + 90);
  text("Use BACKSPACE to delete", width/2, height/2 + 120);
}

void drawGoldMedal(float x, float y, float size) {
  pushMatrix();
  translate(x, y);
  
  // Medal ribbon - V-shaped red ribbon
  fill(200, 50, 50);
  noStroke();
  triangle(-size*0.3, -size*0.8, 0, -size*0.3, size*0.3, -size*0.8);
  
  // Medal circle background
  fill(255, 215, 0);
  stroke(255, 255, 100);
  strokeWeight(3);
  ellipse(0, 0, size, size);
  
  // Inner circle
  fill(255, 235, 50);
  noStroke();
  ellipse(0, 0, size*0.7, size*0.7);
  
  // Number 1
  fill(200, 150, 0);
  textAlign(CENTER);
  textSize(size*0.4);
  text("1", 0, size*0.15);
  
  popMatrix();
}

void drawSilverMedal(float x, float y, float size) {
  pushMatrix();
  translate(x, y);
  
  // Medal ribbon - V-shaped red ribbon
  fill(200, 50, 50);
  noStroke();
  triangle(-size*0.3, -size*0.8, 0, -size*0.3, size*0.3, -size*0.8);
  
  // Medal circle background
  fill(192, 192, 192);
  stroke(220, 220, 220);
  strokeWeight(3);
  ellipse(0, 0, size, size);
  
  // Inner circle
  fill(220, 220, 220);
  noStroke();
  ellipse(0, 0, size*0.7, size*0.7);
  
  // Number 2
  fill(128, 128, 128);
  textAlign(CENTER);
  textSize(size*0.4);
  text("2", 0, size*0.15);
  
  popMatrix();
}

void drawBronzeMedal(float x, float y, float size) {
  pushMatrix();
  translate(x, y);
  
  // Medal ribbon - V-shaped red ribbon
  fill(200, 50, 50);
  noStroke();
  triangle(-size*0.3, -size*0.8, 0, -size*0.3, size*0.3, -size*0.8);
  
  // Medal circle background
  fill(205, 127, 50);
  stroke(255, 165, 80);
  strokeWeight(3);
  ellipse(0, 0, size, size);
  
  // Inner circle
  fill(255, 165, 80);
  noStroke();
  ellipse(0, 0, size*0.7, size*0.7);
  
  // Number 3
  fill(150, 90, 30);
  textAlign(CENTER);
  textSize(size*0.4);
  text("3", 0, size*0.15);
  
  popMatrix();
}

void drawLeaderboard() {
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(101);
  text("LEADERBOARD", width/2, height/2 - 200);
  
  for (int i = 0; i < 5; i++) {
    float yPos = (height/2 - 100) + i * 80; // Centered with more spacing
    
    // Draw medals for top 3 positions
    if (i == 0) {
      drawGoldMedal(width/2 - 200, yPos - 10, 40);
      fill(255, 215, 0); // Gold
    } else if (i == 1) {
      drawSilverMedal(width/2 - 200, yPos - 10, 40);
      fill(192, 192, 192); // Silver
    } else if (i == 2) {
      drawBronzeMedal(width/2 - 200, yPos - 10, 40);
      fill(205, 127, 50); // Bronze
    } else {
      fill(255); // White
    }
    
    textAlign(CENTER);
    textSize(50);
    text((i + 1) + ". " + playerNames[i] + " - " + highScores[i], width/2, yPos);
  }
  
  fill(200);
  textSize(34);
  text("Press SPACE to restart", width/2, height - 90);
  text("Press ESC to return", width/2, height - 60);
}

 
/****************************************************************************
***************************CODE INSERT END***********************************
*****************************************************************************/
