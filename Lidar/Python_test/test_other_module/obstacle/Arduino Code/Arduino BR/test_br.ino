/**
 * Robot Movement Control and Position Reporting Arduino
 *
 * This Arduino handles:
 * 1. Receiving GX,Y,Z movement commands
 * 2. Controlling the robot's movement
 * 3. Reporting the current position via "POS,x,y,z,targetX,targetY" format
 */

#include <Servo.h>

// Pin definitions
const int LEFT_MOTOR_PIN = 9;
const int RIGHT_MOTOR_PIN = 10;
const int ENCODER_LEFT_A = 2;
const int ENCODER_LEFT_B = 3;
const int ENCODER_RIGHT_A = 4;
const int ENCODER_RIGHT_B = 5;

// Motor control
Servo leftMotor;
Servo rightMotor;

// Position tracking
float posX = 150.0;     // Current X position (cm)
float posY = 100.0;     // Current Y position (cm)
float orientation = 90.0; // Current orientation (degrees)
float targetX = 150.0;  // Target X position
float targetY = 100.0;  // Target Y position

// Movement parameters
const float WHEEL_RADIUS = 3.0;    // Wheel radius in cm
const float WHEEL_BASE = 15.0;     // Distance between wheels in cm
const float ENCODER_TICKS_PER_REV = 1440.0; // Encoder ticks per wheel revolution
const float MAX_SPEED = 30.0;      // Maximum speed in cm/s
const float MAX_TURN_RATE = 90.0;  // Maximum turn rate in degrees/s
const float POSITION_TOLERANCE = 3.0; // Position tolerance in cm
const float ORIENTATION_TOLERANCE = 5.0; // Orientation tolerance in degrees

// Encoder counters
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long prevLeftCount = 0;
long prevRightCount = 0;

// Timing variables
unsigned long lastUpdateTime = 0;
unsigned long lastReportTime = 0;
const unsigned long UPDATE_INTERVAL = 50;     // Motion update interval (ms)
const unsigned long POSITION_REPORT_INTERVAL = 200; // Position report interval (ms)

// Command processing
String inputBuffer = "";
bool commandComplete = false;

// Movement state
bool isMoving = false;
unsigned long movementStartTime = 0;
const unsigned long MOVEMENT_TIMEOUT = 10000; // 10 seconds

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize motor control
  leftMotor.attach(LEFT_MOTOR_PIN);
  rightMotor.attach(RIGHT_MOTOR_PIN);
  stopMotors();

  // Set up encoder interrupts
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateRightEncoder, CHANGE);

  // Initial position report
  reportPosition();

  Serial.println("Movement controller initialized");
}

void loop() {
  // Check for new commands
  handleSerialInput();

  // Update position based on encoder feedback
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    updatePosition();
    lastUpdateTime = currentTime;
  }

  // Periodically report position
  if (currentTime - lastReportTime >= POSITION_REPORT_INTERVAL) {
    reportPosition();
    lastReportTime = currentTime;
  }

  // Update movement
  if (isMoving) {
    updateMovement();

    // Check for movement timeout
    if (currentTime - movementStartTime > MOVEMENT_TIMEOUT) {
      Serial.println("Movement timeout");
      stopMotors();
      isMoving = false;
    }
  }
}

void updateLeftEncoder() {
  // Update left encoder count based on quadrature encoding
  int a = digitalRead(ENCODER_LEFT_A);
  int b = digitalRead(ENCODER_LEFT_B);

  if (a == b) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void updateRightEncoder() {
  // Update right encoder count based on quadrature encoding
  int a = digitalRead(ENCODER_RIGHT_A);
  int b = digitalRead(ENCODER_RIGHT_B);

  if (a != b) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void updatePosition() {
  // Calculate wheel movement since last update
  long deltaLeft = leftEncoderCount - prevLeftCount;
  long deltaRight = rightEncoderCount - prevRightCount;
  prevLeftCount = leftEncoderCount;
  prevRightCount = rightEncoderCount;

  // Convert encoder ticks to distance
  float leftDistance = (deltaLeft / ENCODER_TICKS_PER_REV) * (2.0 * PI * WHEEL_RADIUS);
  float rightDistance = (deltaRight / ENCODER_TICKS_PER_REV) * (2.0 * PI * WHEEL_RADIUS);

  // Calculate robot movement
  float distanceMoved = (leftDistance + rightDistance) / 2.0;
  float angleTurned = (rightDistance - leftDistance) / WHEEL_BASE * (180.0 / PI);

  // Update position and orientation
  orientation += angleTurned;

  // Normalize orientation to 0-360
  while (orientation < 0) orientation += 360.0;
  while (orientation >= 360.0) orientation -= 360.0;

  // Update position based on orientation
  float radians = orientation * PI / 180.0;
  posX += distanceMoved * cos(radians);
  posY += distanceMoved * sin(radians);
}

void updateMovement() {
  // Calculate distance to target
  float dx = targetX - posX;
  float dy = targetY - posY;
  float distanceToTarget = sqrt(dx*dx + dy*dy);

  // Calculate desired heading
  float desiredHeading = atan2(dy, dx) * 180.0 / PI;

  // Normalize to 0-360
  while (desiredHeading < 0) desiredHeading += 360.0;
  while (desiredHeading >= 360.0) desiredHeading -= 360.0;

  // Calculate heading error
  float headingError = desiredHeading - orientation;

  // Normalize heading error to -180 to +180
  if (headingError > 180.0) headingError -= 360.0;
  if (headingError < -180.0) headingError += 360.0;

  // Check if we've reached the target
  if (distanceToTarget <= POSITION_TOLERANCE && abs(headingError) <= ORIENTATION_TOLERANCE) {
    stopMotors();
    isMoving = false;
    Serial.println("Target reached");
    return;
  }

  // Determine if we need to turn or move forward
  float leftSpeed = 0;
  float rightSpeed = 0;

  if (abs(headingError) > ORIENTATION_TOLERANCE) {
    // Turn to face the target
    float turnRate = constrain(headingError, -MAX_TURN_RATE, MAX_TURN_RATE);
    leftSpeed = -turnRate * 0.5;
    rightSpeed = turnRate * 0.5;
  } else {
    // Move toward the target
    float forwardSpeed = min(distanceToTarget, MAX_SPEED);

    // Small adjustment for any heading error
    float turnAdjustment = headingError * 0.5;

    leftSpeed = forwardSpeed - turnAdjustment;
    rightSpeed = forwardSpeed + turnAdjustment;
  }

  // Apply speed to motors
  setMotorSpeeds(leftSpeed, rightSpeed);
}

void setMotorSpeeds(float leftSpeed, float rightSpeed) {
  // Convert speeds to PWM values (90 is stopped for servos)
  // Note: This mapping depends on your specific motor setup
  int leftPWM = map(constrain(leftSpeed, -MAX_SPEED, MAX_SPEED), -MAX_SPEED, MAX_SPEED, 0, 180);
  int rightPWM = map(constrain(rightSpeed, -MAX_SPEED, MAX_SPEED), -MAX_SPEED, MAX_SPEED, 180, 0);

  leftMotor.write(leftPWM);
  rightMotor.write(rightPWM);
}

void stopMotors() {
  leftMotor.write(90);  // Stop value for servo
  rightMotor.write(90); // Stop value for servo
}

void reportPosition() {
  // Send position update in the format: POS,x,y,orientation,targetX,targetY
  Serial.print("POS,");
  Serial.print(posX);
  Serial.print(",");
  Serial.print(posY);
  Serial.print(",");
  Serial.print(orientation);
  Serial.print(",");
  Serial.print(targetX);
  Serial.print(",");
  Serial.println(targetY);
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (inputBuffer.length() > 0) {
        commandComplete = true;
      }
    } else {
      inputBuffer += inChar;
    }
  }

  if (commandComplete) {
    processCommand(inputBuffer);
    inputBuffer = "";
    commandComplete = false;
  }
}

void processCommand(String command) {
  command.trim(); // Remove any whitespace

  if (command.startsWith("G")) {
    // Parse movement command: GX100,Y200,Z90
    float newX = targetX;
    float newY = targetY;
    float newOrientation = orientation;

    // Extract X coordinate if present
    int xIndex = command.indexOf("X");
    if (xIndex >= 0) {
      int commaIndex = command.indexOf(",", xIndex);
      if (commaIndex > 0) {
        newX = command.substring(xIndex + 1, commaIndex).toFloat();
      } else {
        newX = command.substring(xIndex + 1).toFloat();
      }
    }

    // Extract Y coordinate if present
    int yIndex = command.indexOf("Y");
    if (yIndex >= 0) {
      int commaIndex = command.indexOf(",", yIndex);
      if (commaIndex > 0) {
        newY = command.substring(yIndex + 1, commaIndex).toFloat();
      } else {
        newY = command.substring(yIndex + 1).toFloat();
      }
    }

    // Extract Z (orientation) if present
    int zIndex = command.indexOf("Z");
    if (zIndex >= 0) {
      newOrientation = command.substring(zIndex + 1).toFloat();
    }

    // Set new target
    targetX = newX;
    targetY = newY;

    // Start moving
    isMoving = true;
    movementStartTime = millis();

    Serial.print("Moving to X:");
    Serial.print(targetX);
    Serial.print(" Y:");
    Serial.print(targetY);
    Serial.print(" Z:");
    Serial.println(newOrientation);

  } else if (command == "STOP") {
    // Stop movement
    stopMotors();
    isMoving = false;
    Serial.println("Stopped");

  } else {
    // Unknown command
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}