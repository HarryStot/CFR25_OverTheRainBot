#include "Motor_Control.h"
#include "Odometry.h"
#include <math.h> // Include for M_PI

#define ENCAD 19
#define ENCBD 18
#define PWMD 11
#define DIRD 13
#define ENCAG 21
#define ENCBG 20
#define PWMG 3
#define DIRG 12

float L = 0.175 / 2;
float r = 0.065 / 2;

Motor motorR(ENCAD, ENCBD, PWMD, DIRD);
Motor motorL(ENCAG, ENCBG, PWMG, DIRG);
Odometry robot(L, r);

float v = 10.0, w;
float x_goal = 0, y_goal = 1, theta_goal = M_PI/2;
float K1 = 120; // Tuning parameter for angular speed
float eps = 0.05;

enum StateType { STOP, NAVIGATION, SENDPOS };
StateType State = STOP; // Initialize state to STOP
StateType previousState = STOP; // Initialize previousState accordingly

// Helper function to find the end index for a value associated with a command character
int findValueEnd(const String& cmd, int startIndex) {
  int endIndex = cmd.length();
  for (int i = startIndex; i < cmd.length(); i++) {
    char c = cmd.charAt(i);
    // Stop if another command character is found
    if (c == 'G' || c == 'Y' || c == 'Z' || c == 'S' || c == 'P' || c == 'V') {
      endIndex = i;
      break;
    }
  }
  return endIndex;
}


void setup() {
    Serial.begin(115200);

    motorR.init();
    motorL.init();
    attachInterrupt(digitalPinToInterrupt(ENCAD), [] { motorR.readEncoder(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCAG), [] { motorL.readEncoder(); }, RISING);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Process state change commands first
    if (cmd.indexOf('S') != -1) {
      State = STOP;
    } else if (cmd.indexOf('P') != -1) { // Use else if to avoid immediate override if S and P are in the same command
      previousState = State; // Store current state before changing
      State = SENDPOS;
    } else if (State == STOP && (cmd.indexOf('G') != -1 || cmd.indexOf('V') != -1)) {
        // If stopped and a new goal or velocity is received, start navigating again
        State = NAVIGATION;
    }


    // Find command character positions
    int gPos = cmd.indexOf('G');
    int yPos = cmd.indexOf('Y');
    int zPos = cmd.indexOf('Z');
    int vPos = cmd.indexOf('V');

    // Parse Goal (G, Y, Z)
    // Ensure G, Y, Z appear in that order for a valid goal command
    if (gPos != -1 && yPos > gPos && zPos > yPos) {
        int xEnd = yPos; // Value for X ends before Y
        int yEnd = zPos; // Value for Y ends before Z
        int zEnd = findValueEnd(cmd, zPos + 1); // Value for Z ends at the next command or end of string

        // Convert incoming x_goal and y_goal from cm to meters
        x_goal = cmd.substring(gPos + 1, xEnd).toFloat() / 100.0; 
        y_goal = cmd.substring(yPos + 1, yEnd).toFloat() / 100.0; 
        // Convert incoming theta_goal from degrees to radians
        theta_goal = cmd.substring(zPos + 1, zEnd).toFloat() * M_PI / 180.0; 
        // If a new goal is set, ensure we are in NAVIGATION state (unless S was also sent)
        if (State != STOP) {
             State = NAVIGATION;
        }
    }

    // Parse Velocity (V)
    if (vPos != -1) {
      int vEnd = findValueEnd(cmd, vPos + 1);
      v = cmd.substring(vPos + 1, vEnd).toFloat();
       // If a new velocity is set, ensure we are in NAVIGATION state (unless S was also sent)
       if (State != STOP) {
           State = NAVIGATION;
       }
    }
  }

  robot.updateOdometry(motorR.pos, motorL.pos);

  switch (State) {
    case STOP:
      motorR.setMotorSpeed(0);
      motorL.setMotorSpeed(0);
      // Optionally send position once when stopped
      // Serial.print("STOPPED,X:"); Serial.print(robot.x); Serial.print(",Y:"); Serial.print(robot.y); Serial.print(",Z:"); Serial.println(robot.theta);
      break;

    case NAVIGATION:
      // Check if goal is reached before calculating new speeds
      if (sqrt(pow(robot.y - y_goal, 2) + pow(robot.x - x_goal, 2)) < eps) { // Use eps for goal check
          State = STOP;
          motorR.setMotorSpeed(0); // Stop motors immediately upon reaching goal
          motorL.setMotorSpeed(0);                
      } else {
          float thetaref = atan2(y_goal - robot.y, x_goal - robot.x);
          // Normalize angle difference to [-PI, PI]
          float angle_diff = atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
          w = K1 * angle_diff;

          // Basic saturation for angular velocity (optional, adjust limits as needed)
          // float max_w = 2.0; // rad/s
          // w = constrain(w, -max_w, max_w);

          float speedR = (v + w * L) / r;
          float speedL = (v - w * L) / r;

          motorR.setMotorSpeed(speedR);
          motorL.setMotorSpeed(speedL);

          // Send position periodically during navigation
          Serial.print("POS"); Serial.print(",X:");
          Serial.print(robot.x * 100.0); // Convert x to cm
          Serial.print(",Y:");
          Serial.print(robot.y * 100.0); // Convert y to cm
          Serial.print(",Z:");
          Serial.println(robot.theta * 180.0 / M_PI); // Convert theta to degrees
      }
      break;

    case SENDPOS:
      Serial.print("POS"); Serial.print(",X:");
      Serial.print(robot.x * 100.0); // Convert x to cm
      Serial.print(",Y:");
      Serial.print(robot.y * 100.0); // Convert y to cm
      Serial.print(",Z:");
      Serial.println(robot.theta * 180.0 / M_PI); // Convert theta to degrees
      // Restore the state from before SENDPOS was triggered
      State = previousState;
      // If you want it to stay in SENDPOS and continuously send, comment out the line above.
      break;
  }
  // Removed goal check from here as it's now inside the NAVIGATION case
  // Serial.println(State); // Optional: for debugging state changes
}