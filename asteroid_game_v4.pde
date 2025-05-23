// Haptic Asteroid Game - Processing Implementation
// Team 1: Manuel, Nick, Jorge, Giuse

// Game variables
int score = 756;
int lives = 3;
int maxLives = 3;
boolean gameRunning = true;
float gameTime = 0;

// Spaceship variables
PVector spaceshipPos;
PVector spaceshipVel;
float spaceshipSize = 30;
int cargoCollected = 0;
float virtualMass = 1.0;
boolean invincible = false;
float invincibleTimer = 0;

// Game objects
ArrayList<Asteroid> asteroids;
ArrayList<Cargo> cargoItems;
ArrayList<BlackHole> blackHoles;

// Visual effects
ArrayList<Particle> particles;
PVector thrustDirection;
boolean showThrust = false;

// Colors
color backgroundColor = color(0, 10, 30);
color spaceshipColor = color(220, 50, 50);
color thrustColor = color(255, 150, 0);

// Leaderboard
int[] highScores = new int[5]; // Top 5 scores
String[] playerNames = {"AAA", "BBB", "CCC", "DDD", "EEE"};
boolean showLeaderboard = false;
boolean enteringName = false;
String currentPlayerName = "";
int nameIndex = 0;

void setup() {
  size(800, 600);
  
  // Initialize spaceship
  spaceshipPos = new PVector(width/2, height - 80);
  spaceshipVel = new PVector(0, 0);
  thrustDirection = new PVector(0, 0);
  
  // Initialize game objects
  asteroids = new ArrayList<Asteroid>();
  cargoItems = new ArrayList<Cargo>();
  particles = new ArrayList<Particle>();
  blackHoles = new ArrayList<BlackHole>();
  
  // Create initial asteroids
  for (int i = 0; i < 8; i++) { // Increased from 5 to 8 initial asteroids
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
    checkCollisions();
    
    // Draw everything
    drawStars();
    drawBlackHoles();
    drawAsteroids();
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
  // Simple movement - in actual implementation, this would be controlled by pantograph
  // For now, use mouse for demonstration
  PVector target = new PVector(mouseX, mouseY);
  PVector force = PVector.sub(target, spaceshipPos);
  force.mult(0.02 / virtualMass); // Mass affects responsiveness
  
  spaceshipVel.add(force);
  spaceshipVel.mult(0.95); // Damping
  spaceshipPos.add(spaceshipVel);
  
  // Keep spaceship in bounds
  spaceshipPos.x = constrain(spaceshipPos.x, spaceshipSize/2, width - spaceshipSize/2);
  spaceshipPos.y = constrain(spaceshipPos.y, spaceshipSize/2, height - spaceshipSize/2);
  
  // Update thrust direction for visual effect
  thrustDirection = PVector.mult(spaceshipVel, -1);
  thrustDirection.normalize();
  showThrust = spaceshipVel.mag() > 1;
  
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
      spaceshipVel.add(pull);
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
  for (Asteroid ast : asteroids) {
    if (PVector.dist(spaceshipPos, ast.pos) < (spaceshipSize/2 + ast.size/2)) {
      // Collision!
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
  
  // Flicker if invincible
  if (!invincible || frameCount % 10 < 5) {
    // Draw spaceship body
    fill(spaceshipColor);
    noStroke();
    
    // Simple rocket shape
    beginShape();
    vertex(0, -15);
    vertex(-10, 10);
    vertex(-5, 8);
    vertex(0, 15);
    vertex(5, 8);
    vertex(10, 10);
    endShape(CLOSE);
    
    // Draw thrust
    if (showThrust) {
      fill(thrustColor);
      pushMatrix();
      rotate(atan2(thrustDirection.y, thrustDirection.x) + PI/2);
      ellipse(0, 20, 8, 15);
      ellipse(-3, 25, 4, 8);
      ellipse(3, 25, 4, 8);
      popMatrix();
    }
  }
  
  popMatrix();
}

void drawAsteroids() {
  for (Asteroid ast : asteroids) {
    ast.draw();
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
  text("Cargo: " + cargoCollected, 20, height - 60);
  text("Mass: " + nf(virtualMass, 1, 1) + "x", 20, height - 40);
  text("Virtual Mass affects control!", 20, height - 20);
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
  spaceshipPos.set(width/2, height - 80);
  spaceshipVel.set(0, 0);
  asteroids.clear();
  cargoItems.clear();
  particles.clear();
  blackHoles.clear();
  
  // Reinitialize
  for (int i = 0; i < 8; i++) { // Match the initial asteroid count
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
    pos.add(vel);
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
    pos = new PVector(random(100, width-100), random(50, height-200));
    size = random(60, 100);
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
    
    if (distance > 0 && distance < 200) {
      force.normalize();
      float strength = 300.0 / (distance * distance);
      strength = constrain(strength, 0, 0.5);
      force.mult(strength);
      return force;
    }
    return new PVector(0, 0);
  }
  
  void draw() {
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(rotation);
    
    // Draw gravitational field
    noFill();
    for (int i = 1; i <= 5; i++) {
      stroke(100, 50, 200, 50 - i * 8);
      strokeWeight(2);
      ellipse(0, 0, size + i * 20, size + i * 20);
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
