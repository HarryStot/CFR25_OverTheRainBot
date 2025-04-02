/*
 * Motor Control Simulation with Position Feedback
 * This sketch simulates an Arduino controlling a motor and providing position feedback
 * via serial communication without actually connecting to hardware.
 *
 * Commands:
 * GX25,Y84,Z41 - Go to position X=25, Y=84, Z=41 (where Z is theta/angle)
 * S - Stop all motors
 * V42 - Set velocity to 42 (range 0-255)
 */

// Constants
const int MAX_VELOCITY = 255;     // Maximum velocity value
const int UPDATE_INTERVAL = 100;  // Update position every 100ms

// Variables for X, Y positions and Z angle
float currentX = 0.0;
float currentY = 0.0;
float currentZ = 0.0;  // Z represents theta (angle)

float targetX = 0.0;
float targetY = 0.0;
float targetZ = 0.0;

int velocity = 50;  // Default velocity (range 0-255)
bool isMoving = false;

unsigned long lastUpdateTime = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Motor Control Simulation Started");
  Serial.println("Commands:");
  Serial.println("  GX25,Y84,Z41 - Go to position X=25, Y=84, Z=41 (angle)");
  Serial.println("  S - Stop all motors");
  Serial.println("  V42 - Set velocity to 42 (range 0-255)");
  Serial.println("  P - Request current position");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // Update position based on movement towards target
  unsigned long currentTime = millis();
  if (isMoving && (currentTime - lastUpdateTime >= UPDATE_INTERVAL)) {
    updatePosition();
    sendPositionFeedback();
    lastUpdateTime = currentTime;
  }

  // Simulate other processing
  delay(10);  // Small delay to prevent the loop from running too fast
}

void processCommand(String command) {
  if (command.length() > 0) {
    char cmdType = command.charAt(0);

    switch (cmdType) {
      case 'G': // Go to position
        parseGoToCommand(command);
        isMoving = true;
        Serial.println("Moving to target position");
        break;

      case 'S': // Stop
        isMoving = false;
        Serial.println("Movement stopped");
        break;

      case 'V': // Set velocity
        if (command.length() > 1) {
          int newVelocity = command.substring(1).toInt();
          setVelocity(newVelocity);
        }
        break;

      case 'P': // Request position
        sendPositionFeedback();
        break;

      default:
        Serial.println("Unknown command");
    }
  }
}

void parseGoToCommand(String command) {
  // Parse GX25,Y84,Z41 format
  int xIndex = command.indexOf('X');
  int yIndex = command.indexOf('Y');
  int zIndex = command.indexOf('Z');

  if (xIndex != -1) {
    int nextComma = command.indexOf(',', xIndex);
    if (nextComma == -1) nextComma = command.length();
    targetX = command.substring(xIndex + 1, nextComma).toFloat();
  }

  if (yIndex != -1) {
    int nextComma = command.indexOf(',', yIndex);
    if (nextComma == -1) nextComma = command.length();
    targetY = command.substring(yIndex + 1, nextComma).toFloat();
  }

  if (zIndex != -1) {
    int nextComma = command.indexOf(',', zIndex);
    if (nextComma == -1) nextComma = command.length();
    targetZ = command.substring(zIndex + 1, nextComma).toFloat();
    // Normalize angle between 0-360
    targetZ = fmod(targetZ, 360);
    if (targetZ < 0) targetZ += 360;
  }

  Serial.print("Target set to X:");
  Serial.print(targetX);
  Serial.print(" Y:");
  Serial.print(targetY);
  Serial.print(" Z:");
  Serial.println(targetZ);
}

void setVelocity(int newVelocity) {
  // Constrain velocity to valid range
  velocity = constrain(newVelocity, 0, MAX_VELOCITY);

  Serial.print("Velocity set to: ");
  Serial.println(velocity);
}

void updatePosition() {
  // Calculate maximum step size based on velocity
  float stepSize = velocity / 50.0; // Adjust this divisor to tune movement speed

  // Update X position
  if (abs(targetX - currentX) <= stepSize) {
    currentX = targetX;
  } else if (currentX < targetX) {
    currentX += stepSize;
  } else {
    currentX -= stepSize;
  }

  // Update Y position
  if (abs(targetY - currentY) <= stepSize) {
    currentY = targetY;
  } else if (currentY < targetY) {
    currentY += stepSize;
  } else {
    currentY -= stepSize;
  }

  // Update Z (theta) with special handling for angles
  float zDiff = targetZ - currentZ;
  // Handle shortest path around the circle
  if (zDiff > 180) zDiff -= 360;
  if (zDiff < -180) zDiff += 360;

  if (abs(zDiff) <= stepSize) {
    currentZ = targetZ;
  } else if (zDiff > 0) {
    currentZ += stepSize;
  } else {
    currentZ -= stepSize;
  }

  // Normalize Z angle between 0-360
  currentZ = fmod(currentZ, 360);
  if (currentZ < 0) currentZ += 360;

  // Check if we've reached the target
  if (currentX == targetX && currentY == targetY && currentZ == targetZ) {
    isMoving = false;
    Serial.println("Target position reached");
  }
}

void sendPositionFeedback() {
  Serial.print("POS,X:");
  Serial.print(currentX, 1);
  Serial.print(",Y:");
  Serial.print(currentY, 1);
  Serial.print(",Z:");
  Serial.print(currentZ, 1);
  Serial.println();
}