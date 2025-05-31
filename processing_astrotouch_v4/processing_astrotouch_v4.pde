// Haptic Asteroid Game - Processing Implementation with Pantograph Control
// Team 1: Manuel, Nick, Jorge, Giuse
// AstroTouch - Haptic Space Navigation Game
// ENHANCED VERSION - 3D visuals, improved mechanics, full screen

import processing.serial.*;

// Serial communication for encoder motor
Serial arduinoPort;
boolean serialConnected = false;
String serialPortName = "COM7"; // Change to your Arduino port (leader Arduino)

// Pantograph workspace mapping - Updated to encoder motor boundaries
// Encoder motor boundaries: (144,187), (144,250), (262,187), (262,250)
final float ARD_X_MIN = 144;   // mm (encoder left boundary)
final float ARD_X_MAX = 262;   // mm (encoder right boundary)  
final float ARD_Y_MIN = 187;   // mm (encoder top boundary)
final float ARD_Y_MAX = 250;   // mm (encoder bottom boundary)

// Haptic force variables
PVector currentForce = new PVector(0, 0);
PVector previousPantoPos = new PVector(0, 0);
PVector pantoVelocity = new PVector(0, 0);
PVector pantoPos = new PVector(203, 218); // Raw encoder position in mm (center of workspace)

// Force rendering constants - TUNED VALUES
final float ASTEROID_IMPACT_FORCE = 8.0;    // N
final float BLACKHOLE_MAX_FORCE = 12.0;     // N
final float CARGO_MASS_INCREMENT = 0.3;     // mass multiplier per cargo
final float BASE_DAMPING = 0.1;             // N⋅s/mm
final float WALL_STIFFNESS = 1.5;           // N/mm
final float MAX_FORCE = 15.0;               // N - safety limit

// Game states
boolean showPressStart = true;
boolean gameRunning = false;

// Game variables
int score = 0;
int lives = 3;
int maxLives = 3;
float gameTime = 0;

// UFO variables
PVector spaceshipPos;
PVector spaceshipVel;
float spaceshipSize = 60;
int cargoCollected = 0;
float virtualMass = 1.0;
boolean invincible = false;
float invincibleTimer = 0;
float gameSpeedMultiplier = 1.0;
int damageLevel = 0; // 0 = pristine, 1 = damaged, 2 = heavily damaged

// Game objects
ArrayList<Asteroid> asteroids;
ArrayList<Cargo> cargoItems;
ArrayList<BlackHole> blackHoles;
ArrayList<AsteroidFragment> asteroidFragments;

// Visual effects
ArrayList<Particle> particles;
ArrayList<Star> stars;
ArrayList<RaceLight> raceLights;
ArrayList<SmokeParticle> smokeParticles;

// Colors and animation
color backgroundColor = color(0, 10, 30);
color ufoColor = color(150, 100, 255);
color ufoAccentColor = color(255, 150, 255);
float ufoSpinAngle = 0;
float frameSpinAngle = 0;

// Leaderboard
int[] highScores = new int[5];
String[] playerNames = {"AAA", "BBB", "CCC", "DDD", "EEE"};
boolean showLeaderboard = false;
boolean enteringName = false;
String currentPlayerName = "";
int nameIndex = 0;

// Debug visualization
boolean showForceVectors = false;
boolean showDebugInfo = false;

// Communication variables
String serialBuffer = "";
long lastForceTime = 0;
long lastPositionTime = 0;

// Speed scaling variables
int lastSpeedIncrement = 0;
float baseAsteroidSpeed = 1.0;
float baseCargoSpeed = 3.0; // 3x faster initially

void setup() {
  // FULL SCREEN MODE - Using fullScreen() instead of size()
  fullScreen();
  
  // Initialize serial communication for pantograph
  initializeSerial();
  
  // Initialize UFO - start at center
  spaceshipPos = new PVector(width/2, height/2);
  spaceshipVel = new PVector(0, 0);
  
  // Initialize game objects
  asteroids = new ArrayList<Asteroid>();
  cargoItems = new ArrayList<Cargo>();
  particles = new ArrayList<Particle>();
  blackHoles = new ArrayList<BlackHole>();
  asteroidFragments = new ArrayList<AsteroidFragment>();
  stars = new ArrayList<Star>();
  raceLights = new ArrayList<RaceLight>();
  smokeParticles = new ArrayList<SmokeParticle>();
  
  // Create background stars (scaled for full screen)
  for (int i = 0; i < (width * height) / 8000; i++) {
    stars.add(new Star());
  }
  
  // Initialize leaderboard with default scores
  for (int i = 0; i < 5; i++) {
    highScores[i] = (5 - i) * 1000;
  }
  
  println("AstroTouch Processing Game Started");
  println("Screen size: " + width + "x" + height + " (Full Screen)");
  println("Expected Encoder workspace: X(" + ARD_X_MIN + " to " + ARD_X_MAX + ") Y(" + ARD_Y_MIN + " to " + ARD_Y_MAX + ")");
  
  // Calculate encoder workspace dimensions and aspect ratio
  float encoderWidth = ARD_X_MAX - ARD_X_MIN;  // 118 mm
  float encoderHeight = ARD_Y_MAX - ARD_Y_MIN; // 63 mm
  float encoderAspectRatio = encoderWidth / encoderHeight; // ~1.87:1
  float screenAspectRatio = (float)width / (float)height;
  
  println("Encoder workspace: " + encoderWidth + "mm x " + encoderHeight + "mm (aspect ratio: " + nf(encoderAspectRatio, 1, 2) + ":1)");
  println("Screen aspect ratio: " + nf(screenAspectRatio, 1, 2) + ":1");
  println("Workspace is mapped proportionally to full screen");
}

void draw() {
  background(backgroundColor);
  
  if (showPressStart) {
    updateSpaceshipPosition();
    updateStars();
    
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
    updateAsteroidFragments();
    updateStars();
    updateRaceLights();
    updateSmokeParticles();
    checkCollisions();
    updateSpeedScaling();
    
    // Calculate and send haptic forces
    calculateHapticForces();
    sendForcesToArduino();
    
    // Draw everything
    drawStars();
    drawBlackHoles();
    drawAsteroids();
    drawAsteroidFragments();
    drawCargo();
    drawSpaceship();
    drawRaceLights();
    drawParticles();
    drawSmokeParticles();
    drawUI();
    
    // Debug visualization
    if (showForceVectors) {
      drawForceVector();
    }
    if (showDebugInfo) {
      drawDebugInfo();
    }
    
    // Spawn new objects
    if (frameCount % 120 == 0) { // Every 2 seconds
      asteroids.add(new Asteroid());
    }
    
    // Stop spawning cargo after 20x mass multiplier
    if (frameCount % 300 == 0 && cargoItems.size() < 3 && virtualMass < 20.0) {
      cargoItems.add(new Cargo());
    }
    
    if (frameCount % 300 == 0 && blackHoles.size() < 2) { // Every 5 seconds
      blackHoles.add(new BlackHole());
    }
    
    // Update score
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
}

void initializeSerial() {
  try {
    println("Available serial ports:");
    for (int i = 0; i < Serial.list().length; i++) {
      println(i + ": " + Serial.list()[i]);
    }
    
    // Match Arduino leader baud rate exactly
    arduinoPort = new Serial(this, serialPortName, 115200);
    arduinoPort.bufferUntil('\n'); // Wait for complete lines
    serialConnected = true;
    println("Connected to Arduino Leader on " + serialPortName);
    
    // Give Arduino time to initialize
    delay(2000);
    
  } catch (Exception e) {
    println("Failed to connect to Arduino: " + e.getMessage());
    println("Using mouse input as fallback");
    serialConnected = false;
  }
}

// CRITICAL: This function receives data from Arduino
void serialEvent(Serial port) {
  try {
    String data = port.readStringUntil('\n');
    if (data != null) {
      data = data.trim();
      
      // Parse space-separated format: "x.xxx y.yyy"
      String[] values = data.split(" ");
      if (values.length == 2) {
        float x = Float.parseFloat(values[0]);
        float y = Float.parseFloat(values[1]);
        
        // Update pantograph position
        pantoPos.set(x, y);
        lastPositionTime = millis();
        
        if (showDebugInfo) {
          println("Received position: (" + x + ", " + y + ")");
        }
      }
    }
  } catch (Exception e) {
    if (showDebugInfo) {
      println("Serial parsing error: " + e.getMessage());
    }
  }
}

void updateSpaceshipPosition() {
  // Use mouse as fallback if no serial connection
  if (!serialConnected || (millis() - lastPositionTime > 500)) {
    // SIMPLE DIRECT MAPPING: mouse position directly to screen position
    spaceshipPos.x = mouseX;
    spaceshipPos.y = mouseY;
    
    // Also update pantoPos for force calculations (convert screen back to pantograph coords)
    pantoPos.x = map(mouseX, 0, width, ARD_X_MIN, ARD_X_MAX);
    pantoPos.y = map(mouseY, 0, height, ARD_Y_MIN, ARD_Y_MAX);
  } else {
    // Map pantograph position to screen coordinates
    // INVERTED Y-DIRECTION: Higher pantograph Y should be higher screen Y
    PVector targetPos = new PVector(
      map(pantoPos.x, ARD_X_MIN, ARD_X_MAX, 0, width),
      map(pantoPos.y, ARD_Y_MIN, ARD_Y_MAX, 0, height) // FIXED: ARD_Y_MIN maps to top of screen (0)
    );
    spaceshipPos.set(targetPos);
  }
  
  // Store previous position for velocity calculation
  PVector previousPos = spaceshipPos.copy();
  
  // Keep spaceship in bounds
  spaceshipPos.x = constrain(spaceshipPos.x, spaceshipSize/2, width - spaceshipSize/2);
  spaceshipPos.y = constrain(spaceshipPos.y, spaceshipSize/2, height - spaceshipSize/2);
  
  // Calculate velocity for visual effects and haptics
  PVector newVel = PVector.sub(spaceshipPos, previousPos);
  spaceshipVel.lerp(newVel, 0.3);
  
  // Calculate pantograph velocity for haptic damping
  PVector newPantoVel = PVector.sub(pantoPos, previousPantoPos);
  newPantoVel.mult(60); // Convert to mm/s (assuming 60 FPS)
  pantoVelocity.lerp(newPantoVel, 0.5); // Smooth velocity
  previousPantoPos = pantoPos.copy();
  
  // Create race lights when moving fast
  if (spaceshipVel.mag() > 2.0) {
    createRaceLight();
  }
  
  // Only update game mechanics if game is running
  if (gameRunning) {
    // Increase game difficulty over time
    gameSpeedMultiplier = 1.0 + (gameTime / 60.0) * 0.1;
    
    // Update invincibility
    if (invincible) {
      invincibleTimer -= 1.0/60.0;
      if (invincibleTimer <= 0) {
        invincible = false;
      }
    }
    
    // Update virtual mass based on cargo
    virtualMass = 1.0 + cargoCollected * CARGO_MASS_INCREMENT;
    
    // Create smoke particles if damaged
    if (damageLevel > 0 && frameCount % (10 - damageLevel * 3) == 0) {
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
}

void updateSpeedScaling() {
  // Check for speed increases at 3x, 6x, 9x mass increments
  int currentIncrement = (int)(virtualMass / 3.0);
  if (currentIncrement > lastSpeedIncrement) {
    baseAsteroidSpeed *= 2.0;
    baseCargoSpeed *= 2.0;
    lastSpeedIncrement = currentIncrement;
    
    // Visual feedback for speed increase
    createExplosion(spaceshipPos, color(255, 255, 0));
  }
}

void createSmokeParticle() {
  PVector smokePos = spaceshipPos.copy();
  smokePos.add(random(-spaceshipSize/3, spaceshipSize/3), random(-spaceshipSize/3, spaceshipSize/3));
  PVector smokeVel = new PVector(random(-0.5, 0.5), random(-2, -0.5));
  smokeParticles.add(new SmokeParticle(smokePos, smokeVel));
}

void calculateHapticForces() {
  currentForce.set(0, 0);
  
  if (!gameRunning) {
    // Still apply wall forces in menu to keep user in workspace
    PVector wallForce = calculateWallForces();
    currentForce.add(wallForce);
    return;
  }
  
  // 1. Asteroid collision forces - Spring-damper with exponential decay
  for (Asteroid ast : asteroids) {
    float dist = PVector.dist(spaceshipPos, ast.pos);
    if (dist < (spaceshipSize/2 + ast.size/2 + 20)) { // Extended collision zone
      PVector collisionForce = calculateAsteroidForce(ast);
      currentForce.add(collisionForce);
    }
  }
  
  // 2. Black hole gravitational forces - Inverse square law
  for (BlackHole bh : blackHoles) {
    PVector gravityForce = calculateBlackHoleForce(bh);
    currentForce.add(gravityForce);
  }
  
  // 3. Cargo mass effect - Velocity-proportional damping
  PVector dampingForce = calculateMassDamping();
  currentForce.add(dampingForce);
  
  // 4. Virtual walls - Linear spring boundaries
  PVector wallForce = calculateWallForces();
  currentForce.add(wallForce);
  
  // Limit maximum force for safety
  if (currentForce.mag() > MAX_FORCE) {
    currentForce.normalize();
    currentForce.mult(MAX_FORCE);
  }
}

PVector calculateAsteroidForce(Asteroid ast) {
  // Convert screen positions to pantograph coordinates
  float astX = map(ast.pos.x, 0, width, ARD_X_MIN, ARD_X_MAX);
  float astY = map(ast.pos.y, 0, height, ARD_Y_MIN, ARD_Y_MAX); // FIXED: Direct mapping to match screen mapping
  
  PVector astPanto = new PVector(astX, astY);
  PVector forceDir = PVector.sub(pantoPos, astPanto);
  float distance = forceDir.mag();
  
  if (distance < 0.1) distance = 0.1; // Avoid division by zero
  
  // Spring-like repulsion force (F = k*penetration + F_impact*exp(-d/λ))
  float springConstant = 2.0; // N/mm
  float penetration = max(0, 30 - distance); // 30mm collision radius
  
  forceDir.normalize();
  forceDir.mult(springConstant * penetration + ASTEROID_IMPACT_FORCE * exp(-distance/10));
  
  return forceDir;
}

PVector calculateBlackHoleForce(BlackHole bh) {
  // Convert screen position to pantograph coordinates
  float bhX = map(bh.pos.x, 0, width, ARD_X_MIN, ARD_X_MAX);
  float bhY = map(bh.pos.y, 0, height, ARD_Y_MIN, ARD_Y_MAX); // FIXED: Direct mapping to match screen mapping
  
  PVector bhPanto = new PVector(bhX, bhY);
  PVector forceDir = PVector.sub(bhPanto, pantoPos);
  float distance = forceDir.mag();
  
  if (distance < 1.0) distance = 1.0; // Avoid singularity
  
  // Inverse square law gravitational force (F = G/r²)
  float gravitationalConstant = 800000.0; // N⋅mm²
  float forceMagnitude = gravitationalConstant / (distance * distance);
  forceMagnitude = constrain(forceMagnitude, 0, BLACKHOLE_MAX_FORCE);
  
  forceDir.normalize();
  forceDir.mult(forceMagnitude);
  
  return forceDir;
}

PVector calculateMassDamping() {
  // Velocity-proportional damping with mass scaling (F = -c*v*m)
  float totalDamping = BASE_DAMPING + cargoCollected * 0.05; // N⋅s/mm
  
  PVector dampingForce = pantoVelocity.copy();
  dampingForce.mult(-totalDamping * virtualMass);
  
  return dampingForce;
}

PVector calculateWallForces() {
  PVector wallForce = new PVector(0, 0);
  float wallMargin = 5.0; // mm
  
  // Linear spring forces at workspace boundaries
  if (pantoPos.x < ARD_X_MIN + wallMargin) {
    wallForce.x = WALL_STIFFNESS * (ARD_X_MIN + wallMargin - pantoPos.x);
  }
  if (pantoPos.x > ARD_X_MAX - wallMargin) {
    wallForce.x = WALL_STIFFNESS * (ARD_X_MAX - wallMargin - pantoPos.x);
  }
  if (pantoPos.y < ARD_Y_MIN + wallMargin) {
    wallForce.y = WALL_STIFFNESS * (ARD_Y_MIN + wallMargin - pantoPos.y);
  }
  if (pantoPos.y > ARD_Y_MAX - wallMargin) {
    wallForce.y = WALL_STIFFNESS * (ARD_Y_MAX - wallMargin - pantoPos.y);
  }
  
  return wallForce;
}

void sendForcesToArduino() {
  if (!serialConnected) return;
  
  // Send force components to Arduino in exact format expected
  // Arduino expects: "fx fy\n" (space-separated, newline terminated)
  String forceData = nf(currentForce.x, 1, 3) + " " + nf(currentForce.y, 1, 3) + "\n";
  
  try {
    arduinoPort.write(forceData);
    lastForceTime = millis();
    
    if (showDebugInfo && frameCount % 30 == 0) { // Debug every 0.5 seconds
      println("Sent force: " + forceData.trim());
    }
  } catch (Exception e) {
    println("Error sending forces: " + e.getMessage());
  }
}

void drawForceVector() {
  // Debug visualization of current force
  if (currentForce.mag() > 0.1) {
    stroke(255, 0, 0);
    strokeWeight(3);
    PVector forceEnd = PVector.mult(currentForce, 20); // Scale for visibility
    line(spaceshipPos.x, spaceshipPos.y, spaceshipPos.x + forceEnd.x, spaceshipPos.y + forceEnd.y);
    
    // Force magnitude text
    fill(255, 0, 0);
    textAlign(LEFT);
    text("Force: " + nf(currentForce.mag(), 1, 2) + " N", 10, height - 90); // Scaled text position
  }
}

void drawDebugInfo() {
  fill(255, 255, 0);
  textAlign(LEFT);
  textSize(18); // 50% larger
  text("Encoder Debug Info:", 10, height - 270);
  text("Position: (" + nf(pantoPos.x, 1, 1) + ", " + nf(pantoPos.y, 1, 1) + ") mm", 10, height - 248);
  text("Velocity: (" + nf(pantoVelocity.x, 1, 1) + ", " + nf(pantoVelocity.y, 1, 1) + ") mm/s", 10, height - 225);
  text("Force: (" + nf(currentForce.x, 1, 2) + ", " + nf(currentForce.y, 1, 2) + ") N", 10, height - 203);
  text("Virtual Mass: " + nf(virtualMass, 1, 2), 10, height - 180);
  text("Serial: " + (serialConnected ? "Connected" : "Disconnected"), 10, height - 158);
  text("Screen Size: " + width + "x" + height, 10, height - 135);
  
  // MOUSE DEBUG INFO
  fill(0, 255, 255);
  text("Mouse Debug:", 10, height - 105);
  text("Mouse: (" + mouseX + ", " + mouseY + ")", 10, height - 83);
  text("Spaceship: (" + nf(spaceshipPos.x, 1, 1) + ", " + nf(spaceshipPos.y, 1, 1) + ")", 10, height - 60);
  text("Y-axis: Inverted (controller up → UFO up)", 10, height - 38);
}

void createRaceLight() {
  // Create race light trail behind the UFO
  if (frameCount % 3 == 0) {
    PVector lightPos = spaceshipPos.copy();
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
    
    if (bh.isOffScreen() || bh.shouldRemove()) {
      blackHoles.remove(i);
    } else {
      // Check for asteroid collisions with black hole center
      for (int j = asteroids.size() - 1; j >= 0; j--) {
        Asteroid ast = asteroids.get(j);
        float dist = PVector.dist(ast.pos, bh.pos);
        if (dist < (bh.size * 0.125 + ast.size/2)) { // Black hole center radius + asteroid radius
          // Asteroid gets sucked in
          createExplosion(ast.pos, color(150, 50, 200));
          asteroids.remove(j);
        }
      }
      
      // Apply gravitational pull to spaceship (visual effect only)
      PVector pull = bh.getGravitationalPull(spaceshipPos);
      pull.mult(3.0); // Visual amplification
      
      spaceshipPos.add(pull);
      spaceshipVel.add(PVector.mult(pull, 0.8));
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

void checkCollisions() {
  if (!gameRunning) return;
  
  // Check asteroid collisions (even if invincible, still allow black hole suction)
  if (!invincible) {
    for (int j = asteroids.size() - 1; j >= 0; j--) {
      Asteroid ast = asteroids.get(j);
      if (PVector.dist(spaceshipPos, ast.pos) < (spaceshipSize/2 + ast.size/2)) {
        breakAsteroid(ast);
        asteroids.remove(j);
        
        lives--;
        damageLevel = maxLives - lives; // Increase damage level
        createExplosion(spaceshipPos, color(255, 100, 100));
        if (lives <= 0) {
          gameRunning = false;
          checkHighScore();
        } else {
          invincible = true;
          invincibleTimer = 2.0;
        }
        break;
      }
    }
  }
  
  // Check cargo collection - larger hitbox for easier collection
  for (int i = cargoItems.size() - 1; i >= 0; i--) {
    Cargo cargo = cargoItems.get(i);
    if (PVector.dist(spaceshipPos, cargo.pos) < (spaceshipSize/2 + cargo.size/2 + 15)) {
      cargoCollected++;
      score += 50;
      createExplosion(cargo.pos, color(255, 255, 100));
      cargoItems.remove(i);
    }
  }
  
  // Check black hole collision (always active, even if invincible from asteroid)
  for (BlackHole bh : blackHoles) {
    if (PVector.dist(spaceshipPos, bh.pos) < bh.eventHorizon) {
      gameRunning = false;
      checkHighScore();
      break;
    }
  }
}

void drawSpaceship() {
  pushMatrix();
  translate(spaceshipPos.x, spaceshipPos.y);
  
  // Flicker if invincible
  if (!invincible || frameCount % 10 < 5) {
    // Draw 3D UFO with improved lighting and gradients
    
    // Shadow/depth layer (bottom)
    fill(50, 30, 80);
    noStroke();
    ellipse(2, 4, spaceshipSize * 1.1, spaceshipSize * 0.9);
    
    // Main hull with gradient effect
    for (int i = 8; i >= 0; i--) {
      float alpha = map(i, 0, 8, 255, 100);
      float size = map(i, 0, 8, spaceshipSize, spaceshipSize * 0.6);
      fill(150 + i * 10, 100 + i * 8, 255 - i * 15, alpha);
      ellipse(-i * 0.5, -i * 0.8, size, size * 0.85);
    }
    
    // Outer rim with metallic gradient
    stroke(200, 180, 255);
    strokeWeight(3);
    fill(120, 80, 200);
    ellipse(0, 0, spaceshipSize, spaceshipSize * 0.85);
    
    // Inner hull detail with depth
    fill(80, 60, 160);
    stroke(140, 120, 220);
    strokeWeight(2);
    ellipse(-1, -1, spaceshipSize * 0.7, spaceshipSize * 0.6);
    
    // Central cockpit dome with glass effect
    for (int i = 3; i >= 0; i--) {
      float domeAlpha = map(i, 0, 3, 255, 150);
      float domeSize = map(i, 0, 3, spaceshipSize * 0.35, spaceshipSize * 0.25);
      fill(180 + i * 20, 120 + i * 20, 255, domeAlpha);
      ellipse(-i * 0.3, -i * 0.5, domeSize, domeSize * 0.85);
    }
    
    // Cockpit window with reflection
    fill(220, 200, 255, 200);
    noStroke();
    ellipse(-1, -2, spaceshipSize * 0.2, spaceshipSize * 0.17);
    fill(255, 255, 255, 150);
    ellipse(-2, -3, spaceshipSize * 0.08, spaceshipSize * 0.06);
    
    // Damage effects based on damage level
    if (damageLevel > 0) {
      // Add cracks and damage marks
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
      
      // Add burn marks
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
    
    // Spinning frame with arrows (3D effect)
    pushMatrix();
    rotate(frameSpinAngle);
    
    // Frame shadow
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
    
    // Frame highlight
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
    
    // 3D arrows with depth
    for (int i = 0; i < 8; i++) {
      float angle = TWO_PI * i / 8;
      float arrowRadius = spaceshipSize * 0.55;
      float arrowX = cos(angle) * arrowRadius;
      float arrowY = sin(angle) * arrowRadius * 0.85;
      
      pushMatrix();
      translate(arrowX, arrowY);
      rotate(angle + PI/2);
      
      // Arrow shadow
      stroke(100, 70, 180);
      strokeWeight(3);
      line(-4 + 1, 1, 4 + 1, 1);
      line(2 + 1, -2 + 1, 4 + 1, 1);
      line(2 + 1, 2 + 1, 4 + 1, 1);
      
      // Arrow highlight
      stroke(200, 150, 255);
      strokeWeight(2);
      line(-4, 0, 4, 0);
      line(2, -2, 4, 0);
      line(2, 2, 4, 0);
      popMatrix();
    }
    
    popMatrix();
    
    // Spinning elements with 3D lighting
    pushMatrix();
    rotate(ufoSpinAngle);
    
    // Outer ring lights with depth
    for (int i = 0; i < 12; i++) {
      float angle = TWO_PI * i / 12;
      float lightX = cos(angle) * (spaceshipSize * 0.35);
      float lightY = sin(angle) * (spaceshipSize * 0.3);
      
      float brightness = 150 + sin(frameCount * 0.1 + i * 0.5) * 105;
      
      // Light glow (back layer)
      fill(255, brightness, 255, 80);
      noStroke();
      ellipse(lightX + 1, lightY + 1, 12, 12);
      
      // Main light
      fill(255, brightness, 255);
      ellipse(lightX, lightY, 6, 6);
      
      // Light highlight
      fill(255, 255, 255, brightness);
      ellipse(lightX - 1, lightY - 1, 3, 3);
    }
    
    popMatrix();
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

void drawSmokeParticles() {
  for (SmokeParticle smoke : smokeParticles) {
    smoke.draw();
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
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(72); // 50% larger
  text("AstroTouch", width/2, height/2 - 150);
  
  if (frameCount % 60 < 30) {
    fill(100, 255, 100);
    textSize(36); // 50% larger
    text("Press SPACE to Start", width/2, height/2 - 60);
  }
  
  fill(200);
  textSize(24); // 50% larger
  text("Move the UFO around to position it in your workspace", width/2, height/2);
  text("Use the encoder motor to control the UFO", width/2, height/2 + 30);
  
  fill(150);
  textSize(21); // 50% larger
  text("Collect cargo, avoid asteroids and black holes", width/2, height/2 + 90);
  text("Press L for leaderboard", width/2, height/2 + 120);
  text("Press F to toggle force vectors, D for debug info", width/2, height/2 + 150);
  text("After 20x mass multiplier, no more cargo will spawn!", width/2, height/2 + 180);
  
  fill(serialConnected ? color(100, 255, 100) : color(255, 100, 100));
  textSize(18); // 50% larger
  text("Encoder Motor: " + (serialConnected ? "Connected" : "Disconnected (using mouse)"), width/2, height/2 + 210);
  text("Screen: " + width + "x" + height + " pixels (Full Screen)", width/2, height/2 + 240);
}

void drawUI() {
  fill(100, 255, 100);
  textSize(36); // 50% larger
  textAlign(LEFT);
  text("Score: " + nf(score, 5), 30, 60);
  
  // Draw lives as hearts (50% larger)
  float heartX = width - 300;
  float heartY = 60;
  
  for (int i = 0; i < maxLives; i++) {
    if (i < lives) {
      fill(255, 50, 50);
    } else {
      fill(100, 30, 30);
    }
    drawHeart(heartX + i * 53, heartY, 30); // 50% larger hearts
  }
  
  // Draw cargo counter
  fill(255, 200, 100);
  textSize(24); // 50% larger
  textAlign(LEFT);
  text("Cargo: " + cargoCollected, 30, height - 180);
  text("Mass: " + nf(virtualMass, 1, 1) + "x", 30, height - 150);
  text("Speed: " + nf(gameSpeedMultiplier, 1, 1) + "x", 30, height - 120);
  text("Encoder Control Active", 30, height - 90);
  
  // Mass multiplier warning
  if (virtualMass >= 18.0) {
    fill(255, 100, 100);
    textAlign(CENTER);
    textSize(30);
    text("WARNING: No more cargo after 20x mass!", width/2, 100);
  }
  
  fill(150, 150, 255);
  textSize(18); // 50% larger
  textAlign(LEFT);
  text("Encoder Pos: (" + nf(pantoPos.x, 1, 1) + ", " + nf(pantoPos.y, 1, 1) + ") mm", 30, height - 60);
  text("Force: (" + nf(currentForce.x, 1, 2) + ", " + nf(currentForce.y, 1, 2) + ") N", 30, height - 38);
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
  textSize(72); // 50% larger
  text("GAME OVER", width/2, height/2 - 75);
  
  fill(255);
  textSize(36); // 50% larger
  text("Final Score: " + score, width/2, height/2);
  text("Cargo Collected: " + cargoCollected, width/2, height/2 + 45);
  text("Final Mass Multiplier: " + nf(virtualMass, 1, 1) + "x", width/2, height/2 + 90);
  
  textSize(24); // 50% larger
  text("Press R to restart", width/2, height/2 + 150);
  text("Press L to view leaderboard", width/2, height/2 + 180);
}

void drawNameEntry() {
  background(backgroundColor);
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(54); // 50% larger
  text("NEW HIGH SCORE!", width/2, height/2 - 150);
  
  fill(255);
  textSize(36); // 50% larger
  text("Score: " + score, width/2, height/2 - 90);
  text("Enter your name:", width/2, height/2 - 30);
  
  fill(100, 255, 100);
  textSize(48); // 50% larger
  String displayName = currentPlayerName;
  if (frameCount % 60 < 30) displayName += "_";
  text(displayName, width/2, height/2 + 30);
  
  fill(200);
  textSize(24); // 50% larger
  text("Press ENTER when done", width/2, height/2 + 90);
  text("Use BACKSPACE to delete", width/2, height/2 + 120);
}

void drawLeaderboard() {
  background(backgroundColor);
  fill(255, 255, 100);
  textAlign(CENTER);
  textSize(72); // 50% larger
  text("LEADERBOARD", width/2, 120);
  
  for (int i = 0; i < 5; i++) {
    if (i == 0) fill(255, 215, 0); // Gold
    else if (i == 1) fill(192, 192, 192); // Silver
    else if (i == 2) fill(205, 127, 50); // Bronze
    else fill(255); // White
    
    textSize(36); // 50% larger
    text((i + 1) + ". " + playerNames[i] + " - " + highScores[i], width/2, 225 + i * 60);
  }
  
  fill(200);
  textSize(24); // 50% larger
  text("Press R to restart", width/2, height - 90);
  text("Press ESC to return", width/2, height - 60);
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
      showPressStart = false;
      gameRunning = true;
      
      asteroids.clear();
      cargoItems.clear();
      blackHoles.clear();
      
      for (int i = 0; i < 4; i++) { 
        asteroids.add(new Asteroid());
      }
      
      for (int i = 0; i < 2; i++) {
        cargoItems.add(new Cargo());
      }
    } else if (key == 'l' || key == 'L') {
      showLeaderboard = true;
      showPressStart = false;
    } else if (key == 'f' || key == 'F') {
      showForceVectors = !showForceVectors;
    } else if (key == 'd' || key == 'D') {
      showDebugInfo = !showDebugInfo;
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
    } else if (key == 'r' || key == 'R') {
      restartGame();
    }
  } else {
    if (key == 'r' || key == 'R') {
      restartGame();
    } else if (key == 'l' || key == 'L') {
      showLeaderboard = true;
    } else if (key == 'f' || key == 'F') {
      showForceVectors = !showForceVectors;
    } else if (key == 'd' || key == 'D') {
      showDebugInfo = !showDebugInfo;
    }
  }
}

void restartGame() {
  score = 0;
  lives = 3;
  cargoCollected = 0;
  damageLevel = 0;
  gameRunning = false;
  showPressStart = true;
  showLeaderboard = false;
  enteringName = false;
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
  spaceshipPos.set(width/2, height/2);
  spaceshipVel.set(0, 0);
  currentForce.set(0, 0);
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

// ================== GAME OBJECT CLASSES ==================

class SmokeParticle {
  PVector pos, vel;
  float life, maxLife;
  float size;
  color smokeColor;
  
  SmokeParticle(PVector p, PVector v) {
    pos = p.copy();
    vel = v.copy();
    maxLife = life = random(40, 80);
    size = random(3, 8);
    smokeColor = color(80, 80, 80);
  }
  
  void update() {
    pos.add(vel);
    vel.mult(0.95);
    vel.add(0, -0.1); // Float upward
    life--;
    size += 0.1; // Expand as it rises
  }
  
  boolean isDead() {
    return life <= 0;
  }
  
  void draw() {
    float alpha = map(life, 0, maxLife, 0, 150);
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

class Star {
  PVector pos, vel;
  float brightness, size;
  
  Star() {
    pos = new PVector(random(width), random(height));
    vel = new PVector(0, random(0.15, 0.65));
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
    vel.y = random(0.15, 0.65);
    brightness = random(50, 255);
    size = random(0.5, 2.0);
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
  
  Asteroid() {
    pos = new PVector(random(width), random(-100, -50));
    vel = new PVector(random(-1, 1), random(1, 3) * baseAsteroidSpeed);
    size = random(30, 60);
    rotation = 0;
    rotSpeed = random(-0.05, 0.05);
    
    // Generate jagged vertices
    vertices = new ArrayList<PVector>();
    int numVertices = int(random(8, 16));
    for (int i = 0; i < numVertices; i++) {
      float angle = TWO_PI * i / numVertices;
      float radius = size/2 * random(0.7, 1.3);
      vertices.add(new PVector(cos(angle) * radius, sin(angle) * radius));
    }
    
    // Generate random craters with shadows
    craters = new ArrayList<Crater>();
    int numCraters = int(random(3, 8));
    for (int i = 0; i < numCraters; i++) {
      float angle = random(TWO_PI);
      float dist = random(size * 0.1, size * 0.3);
      float x = cos(angle) * dist;
      float y = sin(angle) * dist;
      float craterSize = random(size * 0.05, size * 0.15);
      craters.add(new Crater(x, y, craterSize));
    }
  }
  
  void update() {
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
    
    // Draw shadow
    fill(20, 15, 10);
    noStroke();
    beginShape();
    for (PVector vertex : vertices) {
      vertex(vertex.x + 2, vertex.y + 3);
    }
    endShape(CLOSE);
    
    // Draw main asteroid body with gradient
    fill(90, 80, 70);
    stroke(60, 50, 40);
    strokeWeight(2);
    beginShape();
    for (PVector vertex : vertices) {
      vertex(vertex.x, vertex.y);
    }
    endShape(CLOSE);
    
    // Add highlight
    fill(110, 100, 90);
    noStroke();
    beginShape();
    for (int i = 0; i < vertices.size(); i++) {
      PVector vertex = vertices.get(i);
      vertex(vertex.x - 1, vertex.y - 1.5);
    }
    endShape(CLOSE);
    
    // Draw craters with shadows
    for (Crater crater : craters) {
      crater.draw();
    }
    
    // Draw motion lines
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
    // Draw crater shadow (offset)
    fill(25, 20, 15);
    noStroke();
    ellipse(x + size * 0.2, y + size * 0.3, size, size * 0.8);
    
    // Draw main crater
    fill(35, 30, 25);
    ellipse(x, y, size, size * 0.8);
    
    // Draw crater highlight
    fill(50, 45, 40);
    ellipse(x - size * 0.1, y - size * 0.1, size * 0.6, size * 0.5);
  }
}

class Cargo {
  PVector pos, vel;
  float size, glow, glowDirection;
  boolean collected = false;
  
  Cargo() {
    pos = new PVector(random(50, width-50), random(-100, -50));
    vel = new PVector(random(-0.5, 0.5), random(0.5, 1.5) * baseCargoSpeed);
    size = 25;
    glow = 0;
    glowDirection = 1;
  }
  
  void update() {
    pos.add(vel);
    
    glow += glowDirection * 0.03;
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
    
    // Enhanced outer glow effect
    fill(255, 200, 100, 30 + glow * 70);
    noStroke();
    ellipse(0, 0, size * 3.5, size * 3.5);
    
    fill(255, 220, 120, 60 + glow * 120);
    ellipse(0, 0, size * 2.5, size * 2.5);
    
    fill(255, 240, 150, 80 + glow * 150);
    ellipse(0, 0, size * 1.8, size * 1.8);
    
    // Draw cargo box
    fill(180, 120, 60);
    stroke(120, 80, 40);
    strokeWeight(2);
    rectMode(CENTER);
    rect(0, 0, size, size);
    
    // Draw "CARGO" text
    fill(255, 240, 150);
    textAlign(CENTER);
    textSize(8);
    text("CARGO", 0, 3);
    
    // Draw radiating lines
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
    
    // Sparkle effects
    fill(255, 255, 200, 100 + glow * 155);
    noStroke();
    for (int i = 0; i < 6; i++) {
      float sparkleAngle = TWO_PI * i / 6 + frameCount * 0.05;
      float sparkleX = cos(sparkleAngle) * (size * 0.6 + glow * 8);
      float sparkleY = sin(sparkleAngle) * (size * 0.6 + glow * 8);
      ellipse(sparkleX, sparkleY, 3 + glow * 2, 3 + glow * 2);
    }
    
    popMatrix();
    
    // Draw dotted line to spaceship if close enough
    if (spaceshipPos != null && PVector.dist(pos, spaceshipPos) < 150) {
      drawDottedLine(pos, spaceshipPos);
    }
  }
  
  void drawDottedLine(PVector start, PVector end) {
    stroke(255, 240, 150, 200);
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

class BlackHole {
  PVector pos;
  float size, eventHorizon;
  float rotation;
  float lifeTime, maxLifeTime;
  
  BlackHole() {
    // Random position, avoiding edges and spaceship spawn area
    do {
      pos = new PVector(random(100, width-100), random(50, height-200));
    } while (spaceshipPos != null && PVector.dist(pos, spaceshipPos) < 250);
    
    size = 85; // 30% bigger than previous size (was 65)
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
    
    if (distance > 5) {
      force.normalize();
      
      float gravitationalConstant = 800000.0;
      float strength = gravitationalConstant / (distance * distance);
      
      strength = constrain(strength, 0, 15.0);
      
      force.mult(strength);
      return force;
    } else {
      force.normalize();
      force.mult(20.0);
      return force;
    }
  }
  
  void draw() {
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(rotation);
    
    // Draw gravitational field
    noFill();
    for (int i = 1; i <= 6; i++) {
      stroke(100, 50, 200, 60 - i * 8);
      strokeWeight(2);
      ellipse(0, 0, size + i * 30, size + i * 30);
    }
    
    // Draw black hole outer ring
    fill(20, 10, 40);
    stroke(80, 40, 160);
    strokeWeight(3);
    ellipse(0, 0, size, size);
    
    // Draw spiral
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
    
    // Draw center dot with purple outline
    fill(0, 0, 0);
    stroke(150, 100, 255);
    strokeWeight(2);
    ellipse(0, 0, size * 0.25, size * 0.25);
    
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
