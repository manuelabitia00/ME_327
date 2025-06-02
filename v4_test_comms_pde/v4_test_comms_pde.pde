import processing.serial.*;

// Constants for window dimensions
final int WINDOW_WIDTH      = 600;  // Updated to match game
final int WINDOW_HEIGHT     = 400;  // Updated to match game

// Serial settings
final String SERIAL_PORT    = "/dev/cu.usbserial-A10PDD99";
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

// Game states
boolean showPressStart = true;
boolean gameRunning = false;

// Game variables
int score = 0;
int lives = 3;
int maxLives = 3;
float gameTime = 0;

// UFO variables (changed from spaceship)
PVector spaceshipPos;
PVector spaceshipVel;
float spaceshipSize = 60; // Decreased from 80 to 60
int cargoCollected = 0;
float virtualMass = 1.0;
boolean invincible = false;
float invincibleTimer = 0;
float gameSpeedMultiplier = 1.0; // For increasing difficulty

// Game objects
ArrayList<Asteroid> asteroids;
ArrayList<Cargo> cargoItems;
ArrayList<BlackHole> blackHoles;
ArrayList<AsteroidFragment> asteroidFragments; // New for asteroid crumbles

// Visual effects
ArrayList<Particle> particles;
ArrayList<Star> stars; // Moving background stars
ArrayList<RaceLight> raceLights; // New racing light effect

// Colors
color backgroundColor = color(0, 10, 30);
color ufoColor = color(150, 100, 255); // Purple UFO
color ufoAccentColor = color(255, 150, 255); // Pink/magenta accent lights
float ufoSpinAngle = 0; // Spinning animation angle
float frameSpinAngle = 0; // New spinning frame angle
float arrowOrbitAngle = 0; // Arrows orbiting around UFO

// Leaderboard
int[] highScores = new int[5]; // Top 5 scores
String[] playerNames = {"AAA", "BBB", "CCC", "DDD", "EEE"};
boolean showLeaderboard = false;
boolean enteringName = false;
String currentPlayerName = "";
int nameIndex = 0;

void settings()
{
  size(WINDOW_WIDTH, WINDOW_HEIGHT);
  //fullScreen();
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
   
   // Initialize UFO - start at center
   spaceshipPos = new PVector(width/2, height/2);
   spaceshipVel = new PVector(0, 0);
   
   // Initialize game objects
   asteroids = new ArrayList<Asteroid>();
   cargoItems = new ArrayList<Cargo>();
   particles = new ArrayList<Particle>();
   blackHoles = new ArrayList<BlackHole>();
   asteroidFragments = new ArrayList<AsteroidFragment>(); // Initialize fragments
   stars = new ArrayList<Star>(); // Initialize stars
   raceLights = new ArrayList<RaceLight>(); // Initialize race lights
   
   // Create background stars
   for (int i = 0; i < 100; i++) {
     stars.add(new Star());
   }
   
   // Initialize leaderboard with default scores
   for (int i = 0; i < 5; i++) {
     highScores[i] = (5 - i) * 1000; // Default scores: 5000, 4000, 3000, 2000, 1000
   }
}

void draw()
{
   background(backgroundColor);

   if (showPressStart) {
     // In press start mode - allow free movement
     updateSpaceshipPosition();
     updateStars();
     
     // Draw everything
     drawStars();
     drawSpaceship();
     drawPressStartUI();
   } else if (gameRunning) {
     gameTime += 1.0/60.0; // Assuming 60 FPS
     
     // Update game logic
     updateSpaceshipPosition();
     updateAsteroids();
     updateCargo();
     updateBlackHoles();
     updateParticles();
     updateAsteroidFragments(); // Update fragments
     updateStars(); // Update moving stars
     updateRaceLights(); // Update race lights
     checkCollisions();
     
     // Draw everything
     drawStars();
     drawBlackHoles();
     drawAsteroids();
     drawAsteroidFragments(); // Draw fragments
     drawCargo();
     drawSpaceship();
     drawRaceLights(); // Draw race lights
     drawParticles();
     drawUI();
     
     // Spawn new objects occasionally
     if (frameCount % 120 == 0) { // Every 2 seconds at 60fps (was 3 seconds)
       asteroids.add(new Asteroid());
     }
     if (frameCount % 300 == 0 && cargoItems.size() < 3) { // Every 5 seconds
       cargoItems.add(new Cargo());
     }
     if (frameCount % 300 == 0 && blackHoles.size() < 2) { // Every 5 seconds, max 2 black holes
       blackHoles.add(new BlackHole());
     }
     
     // Update score (1 point per second)
     if (frameCount % 60 == 0) {
       score++;
     }
   } else if (enteringName) {
     drawNameEntry();
   } else if (showLeaderboard) {
     drawLeaderboard();
   } else {
     drawGameOver();
   }

   // Debug display from original code
   if (DEBUG_MODE && !showPressStart && !showLeaderboard && !enteringName)
   {
      // Map the realworld coordinates to our window. xPos_mm and yPos_mm are global.
      float xPosWindow = (xPos_mm + X_MAX_MM) * (width  / (X_MAX_MM - X_MIN_MM) );
      float yPosWindow = (yPos_mm - Y_MIN_MM) * (height / (Y_MAX_MM - Y_MIN_MM) );
        
      if (xPosWindow < 0)
      {
        xPosWindow = 0;
      }
      else if (xPosWindow > width)
      {
        xPosWindow = width;
      }
      
      if (yPosWindow < 0)
      {
        yPosWindow = 0;
      }
      else if (yPosWindow > height)
      {
        yPosWindow = height;
      }
      
      // Draw debug position indicator (small red dot)
      fill(255, 0, 0);
      noStroke();
      ellipse(xPosWindow, yPosWindow, 8, 8);

      // Always display real world coords when debugging
      fill(255, 255, 0);
      textAlign(LEFT);
      text("real world coords (mm) x=" + nf(xPos_mm, 1, 2) + ",  y=" + nf(yPos_mm, 1, 2), 10, height - 40);
   }
}

void updateSpaceshipPosition() {
  // Map pantograph coordinates to game coordinates if available
  if (xPos_mm != 0 || yPos_mm != 0) {
    // Map the realworld coordinates to our window. xPos_mm and yPos_mm are global.
    float xPosWindow = (xPos_mm + X_MAX_MM) * (width  / (X_MAX_MM - X_MIN_MM) );
    float yPosWindow = (yPos_mm - Y_MIN_MM) * (height / (Y_MAX_MM - Y_MIN_MM) );
      
    if (xPosWindow < 0)
    {
      xPosWindow = 0;
    }
    else if (xPosWindow > width)
    {
      xPosWindow = width;
    }
    
    if (yPosWindow < 0)
    {
      yPosWindow = 0;
    }
    else if (yPosWindow > height)
    {
      yPosWindow = height;
    }
    
    // Store previous position to calculate actual movement direction
    PVector previousPos = spaceshipPos.copy();
    
    // Set position from pantograph input
    spaceshipPos.set(xPosWindow, yPosWindow);
    
    // Calculate velocity for visual effects
    PVector newVel = PVector.sub(spaceshipPos, previousPos);
    newVel.mult(0.3);
    spaceshipVel.lerp(newVel, 0.2);
  } else {
    // Fallback to mouse control when no pantograph input
    PVector targetPos = new PVector(mouseX, mouseY);
    PVector previousPos = spaceshipPos.copy();
    spaceshipPos.set(targetPos);
    
    // Calculate velocity for visual effects
    PVector newVel = PVector.sub(targetPos, previousPos);
    newVel.mult(0.3);
    spaceshipVel.lerp(newVel, 0.2);
  }
  
  // Keep spaceship in bounds
  spaceshipPos.x = constrain(spaceshipPos.x, spaceshipSize/2, width - spaceshipSize/2);
  spaceshipPos.y = constrain(spaceshipPos.y, spaceshipSize/2, height - spaceshipSize/2);
  
  // Create race lights when moving fast
  if (spaceshipVel.mag() > 2.0) {
    createRaceLight();
  }
  
  // Only update game mechanics if game is running
  if (gameRunning) {
    // Increase game difficulty over time
    gameSpeedMultiplier = 1.0 + (gameTime / 60.0) * 0.1; // 10% faster every minute
    
    // Update invincibility
    if (invincible) {
      invincibleTimer -= 1.0/60.0;
      if (invincibleTimer <= 0) {
        invincible = false;
      }
    }
    
    // Update virtual mass based on cargo
    virtualMass = 1.0 + cargoCollected * 0.3;
  }
  
  // Update UFO spinning animation
  ufoSpinAngle += 0.05; // Continuous spinning
  if (ufoSpinAngle > TWO_PI) {
    ufoSpinAngle -= TWO_PI;
  }
  
  // Update frame spinning animation (separate from main spin)
  frameSpinAngle += 0.08; // Slightly faster than main spin
  if (frameSpinAngle > TWO_PI) {
    frameSpinAngle -= TWO_PI;
  }
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
            
            xPos_mm       = -pos[0];
            yPos_mm       = pos[1];
            float P2x     = pos[2];
            float P2y     = pos[3];
            float P4x     = pos[4];
            float P4y     = pos[5];
            float P24norm = pos[6];
            float P2Hnorm = pos[7];
            float P3Hnorm = pos[8];
            
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

// This is a helper function for serialEvent and returns the computed motor forces in Newtons based on cursor position.
// In processing frame of reference (x positive to the right);
float[] getForces(float xPos_mm, float yPos_mm)
{ 
  PVector force = new PVector(0.0, 0.0);
  
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
  ///////////////////////////////////////////////////////////////////////////////////
  
  // append force due to asteroid
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

// Game functions from UFO game
void createRaceLight() {
  // Create race light trail behind the UFO
  if (frameCount % 3 == 0) { // Create every 3 frames when moving fast
    PVector lightPos = spaceshipPos.copy();
    // Add some randomness around the UFO position
    lightPos.add(random(-spaceshipSize/3, spaceshipSize/3), random(-spaceshipSize/3, spaceshipSize/3));
    raceLights.add(new RaceLight(lightPos));
  }
}

void updateAsteroids() {
  for (int i = asteroids.size() - 1; i >= 0; i--) {
    Asteroid ast = asteroids.get(i);
    ast.update();
    if (ast.isOffScreen()) {
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
    cargo.update();
    if (cargo.isOffScreen()) {
      cargoItems.remove(i);
    }
  }
}

void updateBlackHoles() {
  for (int i = blackHoles.size() - 1; i >= 0; i--) {
    BlackHole bh = blackHoles.get(i);
    bh.update();
    
    // Remove black holes that are off screen or have been active too long
    if (bh.isOffScreen() || bh.shouldRemove()) {
      blackHoles.remove(i);
    } else {
      // Apply gravitational pull to spaceship
      PVector pull = bh.getGravitationalPull(spaceshipPos);
      
      // Scale the pull effect to make it EXTREMELY dramatic
      pull.mult(3.0); // Increased from 1.5 to 3.0 - triple amplification!
      
      // Apply much stronger gravitational effects
      spaceshipPos.add(pull);
      spaceshipVel.add(PVector.mult(pull, 0.8)); // Increased from 0.5 to 0.8 for stronger visual feedback
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
      // Reset star to top when it goes off screen
      star.reset();
    }
  }
}

void checkCollisions() {
  if (invincible || !gameRunning) return;
  
  // Check asteroid collisions
  for (int j = asteroids.size() - 1; j >= 0; j--) {
    Asteroid ast = asteroids.get(j);
    if (PVector.dist(spaceshipPos, ast.pos) < (spaceshipSize/2 + ast.size/2)) {
      // Collision! Break asteroid into fragments
      breakAsteroid(ast);
      asteroids.remove(j);
      
      lives--;
      createExplosion(spaceshipPos, color(255, 100, 100));
      if (lives <= 0) {
        gameRunning = false;
        checkHighScore();
      } else {
        invincible = true;
        invincibleTimer = 2.0; // 2 seconds of invincibility
      }
      break;
    }
  }
  
  // Check cargo collection - LARGER HITBOX for easier collection
  for (int i = cargoItems.size() - 1; i >= 0; i--) {
    Cargo cargo = cargoItems.get(i);
    // Increased cargo collection radius from (spaceshipSize/2 + cargo.size/2) to (spaceshipSize/2 + cargo.size/2 + 15)
    if (PVector.dist(spaceshipPos, cargo.pos) < (spaceshipSize/2 + cargo.size/2 + 15)) {
      cargoCollected++;
      score += 50; // Bonus points for cargo
      createExplosion(cargo.pos, color(255, 255, 100));
      cargoItems.remove(i);
    }
  }
  
  // Check black hole collision
  for (BlackHole bh : blackHoles) {
    if (PVector.dist(spaceshipPos, bh.pos) < bh.eventHorizon) {
      gameRunning = false; // Instant death
      checkHighScore();
      break;
    }
  }
}

void drawSpaceship() {
  pushMatrix();
  translate(spaceshipPos.x, spaceshipPos.y);
  
  // No tilting - UFO stays level
  
  // Flicker if invincible
  if (!invincible || frameCount % 10 < 5) {
    // Draw top-down UFO design (classic flying saucer from above)
    
    // Outer rim/hull
    fill(ufoColor);
    stroke(120, 80, 200);
    strokeWeight(3);
    ellipse(0, 0, spaceshipSize, spaceshipSize * 0.85);
    
    // Inner hull detail
    fill(120, 80, 200);
    stroke(100, 60, 180);
    strokeWeight(2);
    ellipse(0, 0, spaceshipSize * 0.7, spaceshipSize * 0.6);
    
    // Central cockpit dome (top-down view)
    fill(180, 120, 255);
    stroke(140, 100, 220);
    strokeWeight(2);
    ellipse(0, 0, spaceshipSize * 0.35, spaceshipSize * 0.3);
    
    // Cockpit window (top-down)
    fill(200, 150, 255, 180);
    noStroke();
    ellipse(0, 0, spaceshipSize * 0.2, spaceshipSize * 0.17);
    
    // Spinning frame with arrows - REPOSITIONED OUTSIDE UFO
    pushMatrix();
    rotate(frameSpinAngle);
    stroke(140, 100, 220);
    strokeWeight(2);
    
    // Circular spinning frame lines
    for (int i = 0; i < 16; i++) {
      float angle = TWO_PI * i / 16;
      float innerRadius = spaceshipSize * 0.25;
      float outerRadius = spaceshipSize * 0.42;
      
      float x1 = cos(angle) * innerRadius;
      float y1 = sin(angle) * innerRadius * 0.85; // Match UFO ellipse
      float x2 = cos(angle) * outerRadius;
      float y2 = sin(angle) * outerRadius * 0.85; // Match UFO ellipse
      
      // Draw the radial line
      line(x1, y1, x2, y2);
    }
    
    // Purple arrows positioned OUTSIDE the UFO contour
    for (int i = 0; i < 8; i++) {
      float angle = TWO_PI * i / 8;
      // Position arrows outside the UFO (beyond the outer edge)
      float arrowRadius = spaceshipSize * 0.55; // Outside the UFO contour
      float arrowX = cos(angle) * arrowRadius;
      float arrowY = sin(angle) * arrowRadius * 0.85;
      
      pushMatrix();
      translate(arrowX, arrowY);
      rotate(angle + PI/2); // Tangent to circle for rotation indication
      
      // Purple arrows matching UFO color scheme
      stroke(150, 100, 255); // Purple to match UFO
      strokeWeight(2);
      line(-4, 0, 4, 0);
      line(2, -2, 4, 0);
      line(2, 2, 4, 0);
      popMatrix();
    }
    
    popMatrix();
    
    // Spinning elements - UFO lights and details that rotate
    pushMatrix();
    rotate(ufoSpinAngle);
    
    // Outer ring lights - spinning around the rim
    fill(ufoAccentColor);
    noStroke();
    for (int i = 0; i < 12; i++) {
      float angle = TWO_PI * i / 12;
      float lightX = cos(angle) * (spaceshipSize * 0.35);
      float lightY = sin(angle) * (spaceshipSize * 0.3);
      
      // Pulsing lights with spinning effect
      float brightness = 150 + sin(frameCount * 0.1 + i * 0.5) * 105;
      fill(255, brightness, 255);
      ellipse(lightX, lightY, 4, 4);
      
      // Light glow
      fill(255, brightness, 255, 80);
      ellipse(lightX, lightY, 8, 8);
    }
    
    // Inner spinning details
    for (int i = 0; i < 8; i++) {
      float angle = TWO_PI * i / 8;
      float detailX = cos(angle) * (spaceshipSize * 0.15);
      float detailY = sin(angle) * (spaceshipSize * 0.13);
      
      fill(200, 120, 255);
      noStroke();
      ellipse(detailX, detailY, 3, 3);
    }
    
    // Spinning energy pattern in center
    stroke(255, 180, 255, 150);
    strokeWeight(1);
    noFill();
    for (int i = 0; i < 4; i++) {
      float spiralAngle = TWO_PI * i / 4;
      float spiralRadius = spaceshipSize * 0.08;
      beginShape();
      for (float a = 0; a < TWO_PI; a += 0.2) {
        float x = cos(a + spiralAngle) * spiralRadius * (a / TWO_PI);
        float y = sin(a + spiralAngle) * spiralRadius * (a / TWO_PI);
        vertex(x, y);
      }
      endShape();
    }
    
    popMatrix();
    
    // Static hull details (don't spin)
    // Hull segments
    stroke(100, 60, 180);
    strokeWeight(1);
    noFill();
    for (int i = 0; i < 6; i++) {
      float segmentAngle = TWO_PI * i / 6;
      float x1 = cos(segmentAngle) * (spaceshipSize * 0.25);
      float y1 = sin(segmentAngle) * (spaceshipSize * 0.21);
      float x2 = cos(segmentAngle) * (spaceshipSize * 0.35);
      float y2 = sin(segmentAngle) * (spaceshipSize * 0.3);
      line(x1, y1, x2, y2);
    }
  }
  
  popMatrix();
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

void breakAsteroid(Asteroid ast) {
  // Create 4-6 fragments from the asteroid
  int numFragments = int(random(4, 7));
  for (int i = 0; i < numFragments; i++) {
    PVector fragPos = ast.pos.copy();
    PVector fragVel = PVector.random2D();
    fragVel.mult(random(1, 4));
    fragVel.add(ast.vel); // Inherit some velocity from original asteroid
    
    float fragSize = ast.size * random(0.2, 0.4);
    asteroidFragments.add(new AsteroidFragment(fragPos, fragVel, fragSize));
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

void drawStars() {
  for (Star star : stars) {
    star.draw();
  }
}

void drawPressStartUI() {
  // Draw "Press Start" text
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(48);
  text("UFO ADVENTURE", width/2, height/2 - 100);
  
  // Blinking "Press SPACE to Start" text
  if (frameCount % 60 < 30) {
    fill(100, 255, 100);
    textSize(24);
    text("Press SPACE to Start", width/2, height/2 - 40);
  }
  
  fill(200);
  textSize(16);
  text("Move the UFO around to position it in your workspace", width/2, height/2);
  text("Use the pantograph to control the UFO", width/2, height/2 + 20);
  
  fill(150);
  textSize(14);
  text("Collect cargo, avoid asteroids and black holes", width/2, height/2 + 60);
  text("Press L for leaderboard", width/2, height/2 + 80);
}

void drawUI() {
  // Draw score
  fill(100, 255, 100);
  textSize(24);
  textAlign(LEFT);
  text("Score: " + nf(score, 5), 20, 40);
  
  // Draw lives as hearts
  float heartX = width - 200;
  float heartY = 40;
  
  for (int i = 0; i < maxLives; i++) {
    if (i < lives) {
      fill(255, 50, 50);
    } else {
      fill(100, 30, 30);
    }
    drawHeart(heartX + i * 35, heartY, 20);
  }
  
  // Draw cargo counter
  fill(255, 200, 100);
  textSize(16);
  textAlign(LEFT);
  text("Cargo: " + cargoCollected, 20, height - 80);
  text("Mass: " + nf(virtualMass, 1, 1) + "x", 20, height - 60);
  text("Speed: " + nf(gameSpeedMultiplier, 1, 1) + "x", 20, height - 40);
  text("Pantograph Control Active", 20, height - 20);
}

void drawHeart(float x, float y, float size) {
  pushMatrix();
  translate(x, y);
  scale(size/20.0);
  noStroke();
  
  // Heart shape using curves
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
  textSize(48);
  text("GAME OVER", width/2, height/2 - 50);
  
  fill(255);
  textSize(24);
  text("Final Score: " + score, width/2, height/2);
  text("Cargo Collected: " + cargoCollected, width/2, height/2 + 30);
  text("Final Multiplier: " + nf(1.0 + cargoCollected * 0.1, 1, 1) + "x", width/2, height/2 + 60);
  
  textSize(16);
  text("Press R to restart", width/2, height/2 + 100);
  text("Press L to view leaderboard", width/2, height/2 + 120);
}

void drawNameEntry() {
  background(backgroundColor);
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(36);
  text("NEW HIGH SCORE!", width/2, height/2 - 100);
  
  fill(255);
  textSize(24);
  text("Score: " + score, width/2, height/2 - 60);
  text("Enter your name:", width/2, height/2 - 20);
  
  // Draw name input
  fill(100, 255, 100);
  textSize(32);
  String displayName = currentPlayerName;
  if (frameCount % 60 < 30) displayName += "_"; // Blinking cursor
  text(displayName, width/2, height/2 + 20);
  
  fill(200);
  textSize(16);
  text("Press ENTER when done", width/2, height/2 + 60);
  text("Use BACKSPACE to delete", width/2, height/2 + 80);
}

void drawLeaderboard() {
  background(backgroundColor);
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(48);
  text("LEADERBOARD", width/2, 80);
  
  // Draw scores
  for (int i = 0; i < 5; i++) {
    if (i == 0) fill(255, 215, 0); // Gold
    else if (i == 1) fill(192, 192, 192); // Silver
    else if (i == 2) fill(205, 127, 50); // Bronze
    else fill(255); // White
    
    textSize(24);
    text((i + 1) + ". " + playerNames[i] + " - " + highScores[i], width/2, 150 + i * 40);
  }
  
  fill(200);
  textSize(16);
  text("Press R to restart", width/2, height - 60);
  text("Press ESC to return", width/2, height - 40);
}

void checkHighScore() {
  // Check if current score qualifies for leaderboard
  for (int i = 0; i < 5; i++) {
    if (score > highScores[i]) {
      enteringName = true;
      nameIndex = i;
      currentPlayerName = "";
      return;
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

void keyPressed() {
  if (showPressStart) {
    if (key == ' ') {
      // Start the game
      showPressStart = false;
      gameRunning = true;
      
      // Initialize game objects for actual gameplay
      asteroids.clear();
      cargoItems.clear();
      blackHoles.clear();
      
      // Create initial asteroids - reduced from 8 to 4
      for (int i = 0; i < 4; i++) { 
        asteroids.add(new Asteroid());
      }
      
      // Create cargo items
      for (int i = 0; i < 2; i++) {
        cargoItems.add(new Cargo());
      }
    } else if (key == 'l' || key == 'L') {
      showLeaderboard = true;
      showPressStart = false;
    }
  } else if (enteringName) {
    if (key == ENTER || key == RETURN) {
      // Finish entering name
      if (currentPlayerName.length() == 0) {
        currentPlayerName = "PLAYER";
      }
      
      // Insert new high score
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
      key = 0; // Prevent Processing from closing
    } else if (key == 'r' || key == 'R') {
      restartGame();
    }
  } else {
    if (key == 'r' || key == 'R') {
      restartGame();
    } else if (key == 'l' || key == 'L') {
      showLeaderboard = true;
    }
  }
}

void restartGame() {
  // Reset game state
  score = 0;
  lives = 3;
  cargoCollected = 0;
  gameRunning = false;
  showPressStart = true;
  showLeaderboard = false;
  enteringName = false;
  gameTime = 0;
  gameSpeedMultiplier = 1.0;
  ufoSpinAngle = 0; // Reset spin
  frameSpinAngle = 0; // Reset frame spin
  spaceshipPos.set(width/2, height/2); // Start at center
  spaceshipVel.set(0, 0);
  asteroids.clear();
  cargoItems.clear();
  particles.clear();
  blackHoles.clear();
  asteroidFragments.clear(); // Clear fragments
  raceLights.clear(); // Clear race lights
  
  // Reset stars
  stars.clear();
  for (int i = 0; i < 100; i++) {
    stars.add(new Star());
  }
}

// RaceLight class for trailing light effect
class RaceLight {
  PVector pos;
  float life, maxLife;
  color lightColor;
  float size;
  
  RaceLight(PVector p) {
    pos = p.copy();
    maxLife = life = random(20, 40); // Short-lived lights
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
    
    // Outer glow
    fill(red(lightColor), green(lightColor), blue(lightColor), alpha * 0.3);
    noStroke();
    ellipse(pos.x, pos.y, currentSize * 3, currentSize * 3);
    
    // Inner light
    fill(red(lightColor), green(lightColor), blue(lightColor), alpha);
    ellipse(pos.x, pos.y, currentSize, currentSize);
    
    // Core bright spot
    fill(255, 255, 255, alpha);
    ellipse(pos.x, pos.y, currentSize * 0.3, currentSize * 0.3);
  }
}

// Star class for moving background
class Star {
  PVector pos, vel;
  float brightness, size;
  
  Star() {
    pos = new PVector(random(width), random(height));
    vel = new PVector(0, random(0.15, 0.65)); // Reduced speed by about 1/3
    brightness = random(50, 255);
    size = random(0.5, 2.0);
  }
  
  void update() {
    pos.add(vel);
  }
  
  boolean isOffScreen() {
    return pos.y > height + 10;
  }
  
  void reset() {
    pos.y = random(-50, -10);
    pos.x = random(width);
    vel.y = random(0.15, 0.65); // Slower reset speed too
    brightness = random(50, 255);
    size = random(0.5, 2.0);
  }
  
  void draw() {
    fill(255, brightness);
    noStroke();
    ellipse(pos.x, pos.y, size, size);
  }
}

// Asteroid class with jagged shapes
class Asteroid {
  PVector pos, vel;
  float size, rotation, rotSpeed;
  int craters;
  ArrayList<PVector> vertices; // For jagged shape
  
  Asteroid() {
    pos = new PVector(random(width), random(-100, -50));
    vel = new PVector(random(-1, 1), random(1, 3));
    size = random(30, 60);
    rotation = 0;
    rotSpeed = random(-0.05, 0.05);
    craters = int(random(3, 8));
    
    // Generate jagged vertices
    vertices = new ArrayList<PVector>();
    int numVertices = int(random(8, 16));
    for (int i = 0; i < numVertices; i++) {
      float angle = TWO_PI * i / numVertices;
      float radius = size/2 * random(0.7, 1.3); // Varying radius for jagged effect
      vertices.add(new PVector(cos(angle) * radius, sin(angle) * radius));
    }
  }
  
  void update() {
    // Apply game speed multiplier to asteroid movement
    PVector scaledVel = PVector.mult(vel, gameSpeedMultiplier);
    pos.add(scaledVel);
    rotation += rotSpeed;
  }
  
  boolean isOffScreen() {
    return pos.y > height + size;
  }
  
  void draw() {
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(rotation);
    
    // Draw jagged asteroid body - darker grey
    fill(70, 60, 50); // Much darker than before (was 120, 100, 80)
    stroke(40, 35, 25); // Darker stroke too
    strokeWeight(2);
    
    // Draw jagged shape
    beginShape();
    for (PVector vertex : vertices) {
      vertex(vertex.x, vertex.y);
    }
    endShape(CLOSE);
    
    // Draw darker craters
    fill(35, 30, 25); // Very dark craters
    noStroke();
    for (int i = 0; i < craters; i++) {
      float angle = TWO_PI * i / craters + rotation * 0.5;
      float dist = size * 0.15;
      float x = cos(angle) * dist;
      float y = sin(angle) * dist;
      ellipse(x, y, size * 0.1, size * 0.1);
    }
    
    // Draw motion lines
    stroke(90, 80, 70); // Darker motion lines
    strokeWeight(1);
    for (int i = 0; i < 4; i++) {
      float lineX = random(-size/3, size/3);
      float lineY = -size/2 - random(10, 20);
      line(lineX, lineY, lineX, lineY + random(5, 15));
    }
    
    popMatrix();
  }
}

// Cargo class with enhanced glow
class Cargo {
  PVector pos, vel;
  float size, glow, glowDirection;
  boolean collected = false;
  
  Cargo() {
    pos = new PVector(random(50, width-50), random(-100, -50));
    vel = new PVector(random(-0.5, 0.5), random(0.5, 1.5));
    size = 25;
    glow = 0;
    glowDirection = 1;
  }
  
  void update() {
    pos.add(vel);
    
    // Animate glow - slower pulse rate
    glow += glowDirection * 0.03; // Reduced from 0.1 to 0.03 for slower pulse
    if (glow > 1 || glow < 0) {
      glowDirection *= -1;
      glow = constrain(glow, 0, 1);
    }
  }
  
  boolean isOffScreen() {
    return pos.y > height + size;
  }
  
  void draw() {
    pushMatrix();
    translate(pos.x, pos.y);
    
    // Enhanced outer glow effect - much more glowy
    fill(255, 200, 100, 30 + glow * 70);
    noStroke();
    ellipse(0, 0, size * 3.5, size * 3.5); // Larger outer glow
    
    fill(255, 220, 120, 60 + glow * 120);
    ellipse(0, 0, size * 2.5, size * 2.5); // Medium glow
    
    fill(255, 240, 150, 80 + glow * 150);
    ellipse(0, 0, size * 1.8, size * 1.8); // Inner glow
    
    // Draw cargo box
    fill(180, 120, 60); // Slightly brighter cargo box
    stroke(120, 80, 40);
    strokeWeight(2);
    rectMode(CENTER);
    rect(0, 0, size, size);
    
    // Draw "CARGO" text - brighter
    fill(255, 240, 150);
    textAlign(CENTER);
    textSize(8);
    text("CARGO", 0, 3);
    
    // Draw enhanced radiating lines
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
    
    // Additional sparkle effects
    fill(255, 255, 200, 100 + glow * 155);
    noStroke();
    for (int i = 0; i < 6; i++) {
      float sparkleAngle = TWO_PI * i / 6 + frameCount * 0.05;
      float sparkleX = cos(sparkleAngle) * (size * 0.6 + glow * 8);
      float sparkleY = sin(sparkleAngle) * (size * 0.6 + glow * 8);
      ellipse(sparkleX, sparkleY, 3 + glow * 2, 3 + glow * 2);
    }
    
    popMatrix();
    
    // Draw dotted line to spaceship if close enough - brighter
    if (spaceshipPos != null && PVector.dist(pos, spaceshipPos) < 150) {
      drawDottedLine(pos, spaceshipPos);
    }
  }
  
  void drawDottedLine(PVector start, PVector end) {
    stroke(255, 240, 150, 200); // Brighter dotted line
    strokeWeight(2);
    
    PVector dir = PVector.sub(end, start);
    float dist = dir.mag();
    dir.normalize();
    
    for (float i = 0; i < dist; i += 10) {
      if (int(i/10) % 2 == 0) {
        PVector p1 = PVector.add(start, PVector.mult(dir, i));
        PVector p2 = PVector.add(start, PVector.mult(dir, min(i + 5, dist)));
        line(p1.x, p1.y, p2.x, p2.y);
      }
    }
  }
}

// BlackHole class with center dot
class BlackHole {
  PVector pos;
  float size, eventHorizon;
  float rotation;
  float lifeTime, maxLifeTime;
  
  BlackHole() {
    // Random position, avoiding edges and spaceship spawn area
    // Ensure black hole doesn't spawn too close to spaceship (increased safety distance)
    do {
      pos = new PVector(random(100, width-100), random(50, height-200));
    } while (spaceshipPos != null && PVector.dist(pos, spaceshipPos) < 250); // Increased from 150 to 250
    
    size = random(50, 80); // Slightly increased from random(45, 75), but less than the original (60, 100)
    eventHorizon = size * 0.4;
    rotation = 0;
    maxLifeTime = lifeTime = random(600, 1200); // 10-20 seconds at 60fps
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
  
  PVector getGravitationalPull(PVector targetPos) {
    PVector force = PVector.sub(pos, targetPos);
    float distance = force.mag();
    
    // EXTREMELY strong inverse square law implementation
    if (distance > 5) { // Avoid division by zero, reduced threshold
      force.normalize();
      
      // MASSIVELY increased gravitational constant - should be very noticeable now
      float gravitationalConstant = 800000.0; // Increased from 200000 to 800000!
      float strength = gravitationalConstant / (distance * distance); // TRUE inverse square law
      
      // Much higher maximum force - dramatic pull when close
      strength = constrain(strength, 0, 15.0); // Increased from 8.0 to 15.0
      
      force.mult(strength);
      return force;
    } else {
      // Very close to black hole - EXTREME pull
      force.normalize();
      force.mult(20.0); // Increased from 10.0 to 20.0 - maximum emergency pull
      return force;
    }
  }
  
  void draw() {
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(rotation);
    
    // Draw gravitational field - enhanced to show stronger effect
    noFill();
    for (int i = 1; i <= 6; i++) {
      stroke(100, 50, 200, 60 - i * 8); // More visible gravitational field
      strokeWeight(2);
      ellipse(0, 0, size + i * 30, size + i * 30); // Larger field rings
    }
    
    // Draw black hole outer ring
    fill(20, 10, 40);
    stroke(80, 40, 160);
    strokeWeight(3);
    ellipse(0, 0, size, size);
    
    // Draw spiral first (behind center dot)
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
    
    // Draw VISIBLE black center dot with purple outline
    fill(0, 0, 0); // Pure black
    stroke(150, 100, 255); // Purple border to match spiral
    strokeWeight(2);
    ellipse(0, 0, size * 0.25, size * 0.25); // Black center with purple outline
    
    // Draw directional arrows
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

// AsteroidFragment class for asteroid crumbles
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
    maxLife = life = random(60, 120); // 1-2 seconds at 60fps
    fragmentColor = color(70, 60, 50); // Darker fragment color to match asteroids
  }
  
  void update() {
    pos.add(vel);
    vel.mult(0.98); // Gradual slowdown
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
    
    // Draw fading fragment - darker color
    fill(red(fragmentColor), green(fragmentColor), blue(fragmentColor), alpha);
    stroke(40, 35, 25, alpha); // Darker stroke
    strokeWeight(1);
    
    // Simple irregular fragment shape
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

// Particle class for explosion effects
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
    size = random(2, 6);
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
    float alpha = map(life, 0, maxLife, 0, 255);
    fill(red(col), green(col), blue(col), alpha);
    noStroke();
    ellipse(pos.x, pos.y, size * (life/maxLife), size * (life/maxLife));
  }
}
