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
float WALL_SPRING_CONST = 5; // N-mm

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
}

void draw()
{
   background(220);

   // Redraw axes every frame
   stroke(0);
   line(0, height/2, width,  height/2);
   line(width/2, 0, width/2, height);

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
   
   // Draw the latest position
   fill(255, 0, 0);
   noStroke();
   ellipse(xPosWindow, yPosWindow, 20, 20);

   // Always display real world coords when debugging
   if (DEBUG_MODE)
   {
      fill(0);
      text("real world coords (mm) x=" + nf(xPos_mm, 1, 2) + ",  y=" + nf(yPos_mm, 1, 2), 10, 20);
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
