import processing.serial.*;

// Constants for window dimensions
final int WINDOW_WIDTH      = 600;
final int WINDOW_HEIGHT     = 400;

// Serial settings
final String SERIAL_PORT    = "COM8";
final int    BAUD_RATE       = 9600;

// Target frame rate
final int TARGET_FRAME_RATE = 60;

// Debug flag
final boolean DEBUG_MODE    = true;

Serial myPort;
float  xPos, yPos;

void settings()
{
   size(WINDOW_WIDTH, WINDOW_HEIGHT);
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
   line(0, WINDOW_HEIGHT/2, WINDOW_WIDTH, WINDOW_HEIGHT/2);
   line(WINDOW_WIDTH/2, 0, WINDOW_WIDTH/2, WINDOW_HEIGHT);

   // Draw the latest position
   fill(255, 0, 0);
   noStroke();
   ellipse(xPos, yPos, 20, 20);

   // Always display coords when debugging
   if (DEBUG_MODE)
   {
      fill(0);
      text("x=" + nf(xPos, 1, 2) + "  y=" + nf(yPos, 1, 2), 10, 20);
   }
}

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
            xPos = Float.parseFloat(parts[0]);
            yPos = Float.parseFloat(parts[1]);

            // Compute forces in separate function
            float[] forces = computeForces(xPos, yPos);
            float f1 = forces[0];
            float f2 = forces[1];

            // Send to Arduino
            String packet = "<"
                          + nf(f1, 1, 3)
                          + ","
                          + nf(f2, 1, 3)
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

// Dummy function to compute motor forces from cursor position
float[] computeForces(float x, float y)
{
   float f1 = x * 0.05;
   float f2 = y * 0.03;
   return new float[] { f1, f2 };
}
