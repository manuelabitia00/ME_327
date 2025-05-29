// Haptic Asteroid Game - Processing Implementation
// Team 1: Manuel, Nick, Jorge, Giuse

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

void setup() {
  size(800, 600);
  
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

void draw() {
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
}

void updateSpaceshipPosition() {
  // FIXED: For pantograph control, the mouse position IS the spaceship position
  // The pantograph directly controls where the UFO is positioned
  PVector targetPos = new PVector(mouseX, mouseY);
  
  // Store previous position to calculate actual movement direction
  PVector previousPos = spaceshipPos.copy();
  
  // Set position directly from pantograph input
  spaceshipPos.set(targetPos);
  
  // Keep spaceship in bounds
  spaceshipPos.x = constrain(spaceshipPos.x, spaceshipSize/2, width - spaceshipSize/2);
  spaceshipPos.y = constrain(spaceshipPos.y, spaceshipSize/2, height - spaceshipSize/2);
  
  // Calculate velocity for visual effects
  PVector newVel = PVector.sub(targetPos, previousPos);
  newVel.mult(0.3);
  spaceshipVel.lerp(newVel, 0.2);
  
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
