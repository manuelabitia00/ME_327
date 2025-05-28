// Haptic Asteroid Game - Processing Implementation
// Team 1: Manuel, Nick, Jorge, Giuse

// Game variables
int score = 756;
int lives = 3;
int maxLives = 3;
boolean gameRunning = true;
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
float ufoTilt = 0; // Tilt angle based on movement direction

// Game objects
ArrayList<Asteroid> asteroids;
ArrayList<Cargo> cargoItems;
ArrayList<BlackHole> blackHoles;
ArrayList<AsteroidFragment> asteroidFragments; // New for asteroid crumbles

// Visual effects
ArrayList<Particle> particles;
boolean showThrust = false;

// Colors
color backgroundColor = color(0, 10, 30);
color ufoColor = color(180, 180, 255);
color ufoAccentColor = color(100, 255, 100);

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
  
  // Create initial asteroids - reduced from 8 to 4
  for (int i = 0; i < 4; i++) { 
    asteroids.add(new Asteroid());
  }
  
  // Create cargo items
  for (int i = 0; i < 2; i++) {
    cargoItems.add(new Cargo());
  }
  
  // Initialize leaderboard with default scores
  for (int i = 0; i < 5; i++) {
    highScores[i] = (5 - i) * 1000; // Default scores: 5000, 4000, 3000, 2000, 1000
  }
}

void draw() {
  background(backgroundColor);
  
  if (gameRunning) {
    gameTime += 1.0/60.0; // Assuming 60 FPS
    
    // Update game logic
    updateSpaceship();
    updateAsteroids();
    updateCargo();
    updateBlackHoles();
    updateParticles();
    updateAsteroidFragments(); // Update fragments
    checkCollisions();
    
    // Draw everything
    drawStars();
    drawBlackHoles();
    drawAsteroids();
    drawAsteroidFragments(); // Draw fragments
    drawCargo();
    drawSpaceship();
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

void updateSpaceship() {
  // FIXED: For pantograph control, the mouse position IS the spaceship position
  // The pantograph directly controls where the UFO is positioned
  PVector targetPos = new PVector(mouseX, mouseY);
  
  // Calculate velocity based on position change (for visual effects)
  PVector newVel = PVector.sub(targetPos, spaceshipPos);
  newVel.mult(0.3); // Smooth the velocity calculation
  spaceshipVel.lerp(newVel, 0.2); // Smooth velocity changes for visual effects
  
  // Set position directly from pantograph input
  spaceshipPos.set(targetPos);
  
  // Keep spaceship in bounds
  spaceshipPos.x = constrain(spaceshipPos.x, spaceshipSize/2, width - spaceshipSize/2);
  spaceshipPos.y = constrain(spaceshipPos.y, spaceshipSize/2, height - spaceshipSize/2);
  
  // Calculate UFO tilt based on horizontal movement
  float horizontalVel = spaceshipVel.x;
  float targetTilt = 0;
  
  if (abs(horizontalVel) > 0.5) { // Only tilt if moving fast enough
    // Tilt proportional to horizontal velocity
    targetTilt = constrain(horizontalVel * 0.15, -PI/6, PI/6); // Max tilt of 30 degrees
  }
  
  // Smooth tilt transition
  ufoTilt = lerp(ufoTilt, targetTilt, 0.15);
  
  // Show thrust effect based on movement speed (for visual feedback)
  showThrust = spaceshipVel.mag() > 1.0;
  
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

void checkCollisions() {
  if (invincible) return;
  
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
  
  // Check cargo collection
  for (int i = cargoItems.size() - 1; i >= 0; i--) {
    Cargo cargo = cargoItems.get(i);
    if (PVector.dist(spaceshipPos, cargo.pos) < (spaceshipSize/2 + cargo.size/2)) {
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
  
  // Apply tilt rotation based on movement direction
  rotate(ufoTilt);
  
  // Flicker if invincible
  if (!invincible || frameCount % 10 < 5) {
    // Draw UFO design (classic flying saucer)
    
    // Main saucer body (bottom dome) - slightly elliptical when tilted
    fill(ufoColor);
    stroke(150, 150, 200);
    strokeWeight(2);
    
    // Scale the saucer slightly based on tilt for 3D effect
    pushMatrix();
    float scaleY = 1.0 - abs(ufoTilt) * 0.3; // Compress vertically when tilted
    scale(1.0, scaleY);
    ellipse(0, 5, spaceshipSize, spaceshipSize * 0.4);
    popMatrix();
    
    // Top dome/cockpit
    fill(200, 200, 255);
    stroke(170, 170, 220);
    strokeWeight(1);
    
    pushMatrix();
    scale(1.0, scaleY);
    ellipse(0, -5, spaceshipSize * 0.6, spaceshipSize * 0.3);
    popMatrix();
    
    // Cockpit window
    fill(100, 150, 255, 150);
    noStroke();
    
    pushMatrix();
    scale(1.0, scaleY);
    ellipse(0, -5, spaceshipSize * 0.4, spaceshipSize * 0.2);
    popMatrix();
    
    // UFO lights around the rim - adjust positions based on tilt
    fill(ufoAccentColor);
    noStroke();
    for (int i = 0; i < 8; i++) {
      float angle = TWO_PI * i / 8;
      float lightX = cos(angle) * (spaceshipSize * 0.4);
      float lightY = sin(angle) * (spaceshipSize * 0.15 * scaleY) + 5;
      
      // Pulsing lights
      float brightness = 150 + sin(frameCount * 0.1 + i) * 105;
      fill(100, brightness, 100);
      ellipse(lightX, lightY, 6, 6);
      
      // Light glow
      fill(100, brightness, 100, 50);
      ellipse(lightX, lightY, 12, 12);
    }
    
    // Central light beam (when moving fast)
    if (showThrust) {
      // Tractor beam effect - adjust for tilt
      fill(100, 255, 100, 80);
      noStroke();
      
      // Main beam - tilted based on UFO orientation
      beginShape();
      vertex(-8, spaceshipSize/2 + 5);
      vertex(8, spaceshipSize/2 + 5);
      vertex(15 + ufoTilt * 20, spaceshipSize/2 + 40); // Tilt the beam
      vertex(-15 + ufoTilt * 20, spaceshipSize/2 + 40);
      endShape(CLOSE);
      
      // Inner beam
      fill(200, 255, 200, 120);
      beginShape();
      vertex(-4, spaceshipSize/2 + 5);
      vertex(4, spaceshipSize/2 + 5);
      vertex(8 + ufoTilt * 10, spaceshipSize/2 + 25);
      vertex(-8 + ufoTilt * 10, spaceshipSize/2 + 25);
      endShape(CLOSE);
      
      // Beam particles - scattered based on tilt
      fill(255, 255, 255, 150);
      for (int i = 0; i < 5; i++) {
        float beamX = random(-10, 10) + ufoTilt * random(5, 15);
        float beamY = spaceshipSize/2 + 10 + random(0, 30);
        ellipse(beamX, beamY, 2, 2);
      }
    }
    
    // Landing gear/stabilizers (small details) - adjust for perspective
    stroke(120, 120, 160);
    strokeWeight(2);
    line(-spaceshipSize * 0.3, 8, -spaceshipSize * 0.35, 12);
    line(spaceshipSize * 0.3, 8, spaceshipSize * 0.35, 12);
    line(0, spaceshipSize * 0.2 * scaleY, 0, spaceshipSize * 0.25 * scaleY);
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
  // Draw some background stars
  fill(255, 100);
  noStroke();
  for (int i = 0; i < 50; i++) {
    float x = (frameCount * 0.1 + i * 123.4) % width;
    float y = (frameCount * 0.05 + i * 67.8) % height;
    ellipse(x, y, 1, 1);
  }
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
  text("Press ESC to return to game over", width/2, height - 40);
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
  if (enteringName) {
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
  gameRunning = true;
  showLeaderboard = false;
  enteringName = false;
  gameTime = 0;
  gameSpeedMultiplier = 1.0;
  ufoTilt = 0; // Reset tilt
  spaceshipPos.set(width/2, height/2); // Start at center
  spaceshipVel.set(0, 0);
  asteroids.clear();
  cargoItems.clear();
  particles.clear();
  blackHoles.clear();
  asteroidFragments.clear(); // Clear fragments
  
  // Reinitialize - reduced asteroids to match new count
  for (int i = 0; i < 4; i++) { 
    asteroids.add(new Asteroid());
  }
  for (int i = 0; i < 2; i++) {
    cargoItems.add(new Cargo());
  }
}

// Asteroid class
class Asteroid {
  PVector pos, vel;
  float size, rotation, rotSpeed;
  int craters;
  
  Asteroid() {
    pos = new PVector(random(width), random(-100, -50));
    vel = new PVector(random(-1, 1), random(1, 3));
    size = random(30, 60);
    rotation = 0;
    rotSpeed = random(-0.05, 0.05);
    craters = int(random(3, 8));
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
    
    // Draw asteroid body
    fill(120, 100, 80);
    stroke(80, 60, 40);
    strokeWeight(2);
    ellipse(0, 0, size, size);
    
    // Draw craters
    fill(80, 60, 40);
    noStroke();
    for (int i = 0; i < craters; i++) {
      float angle = TWO_PI * i / craters + rotation * 0.5;
      float dist = size * 0.2;
      float x = cos(angle) * dist;
      float y = sin(angle) * dist;
      ellipse(x, y, size * 0.15, size * 0.15);
    }
    
    // Draw motion lines
    stroke(150, 130, 110);
    strokeWeight(1);
    for (int i = 0; i < 4; i++) {
      float lineX = random(-size/3, size/3);
      float lineY = -size/2 - random(10, 20);
      line(lineX, lineY, lineX, lineY + random(5, 15));
    }
    
    popMatrix();
  }
}

// Cargo class
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
    
    // Animate glow
    glow += glowDirection * 0.1;
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
    
    // Draw glow effect
    fill(255, 200, 100, 50 + glow * 100);
    noStroke();
    ellipse(0, 0, size * 2, size * 2);
    
    // Draw cargo box
    fill(150, 100, 50);
    stroke(100, 70, 30);
    strokeWeight(2);
    rectMode(CENTER);
    rect(0, 0, size, size);
    
    // Draw "CARGO" text
    fill(255, 200, 100);
    textAlign(CENTER);
    textSize(8);
    text("CARGO", 0, 3);
    
    // Draw radiating lines
    stroke(255, 200, 100, 100 + glow * 155);
    strokeWeight(1);
    for (int i = 0; i < 8; i++) {
      float angle = TWO_PI * i / 8;
      float x1 = cos(angle) * size * 0.8;
      float y1 = sin(angle) * size * 0.8;
      float x2 = cos(angle) * (size * 0.8 + 10 + glow * 5);
      float y2 = sin(angle) * (size * 0.8 + 10 + glow * 5);
      line(x1, y1, x2, y2);
    }
    
    popMatrix();
    
    // Draw dotted line to spaceship if close enough
    if (spaceshipPos != null && PVector.dist(pos, spaceshipPos) < 150) {
      drawDottedLine(pos, spaceshipPos);
    }
  }
  
  void drawDottedLine(PVector start, PVector end) {
    stroke(255, 200, 100, 150);
    strokeWeight(1);
    
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

// BlackHole class
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
    
    // Draw black hole
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
    fragmentColor = color(120, 100, 80);
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
    
    // Draw fading fragment
    fill(red(fragmentColor), green(fragmentColor), blue(fragmentColor), alpha);
    stroke(80, 60, 40, alpha);
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
