#include "Motor_Control.h"
#include "Odometry.h"
#include <math.h> // Include for M_PI

#define ENCAD 19 // Blanc 
#define ENCBD 18 // Vert 
#define PWMD 11  // 
#define DIRD 13  // 
#define ENCAG 21 // Vert 
#define ENCBG 20 // Blanc
#define PWMG 3   // 
#define DIRG 12  // 

float L = 0.175 / 2;
float r = 0.065 / 2;

Motor motorR(ENCAD, ENCBD, PWMD, DIRD);
Motor motorL(ENCAG, ENCBG, PWMG, DIRG);
Odometry robot(L, r);

float v = 10.0, w;
float x_goal = 0, y_goal = 1, theta_goal = M_PI/2;
float K1 = 120; // Tuning parameter for angular speed during navigation
float K_align = 10.0; // Tuning parameter for angular speed during alignment
float eps = 0.05; // Position tolerance
float alignment_tolerance = 0.05; // Angular tolerance for alignment (radians)

// Added debug flag
bool debug = false; 

enum StateType { STOP, ALIGNING, NAVIGATION, SENDPOS }; // Added ALIGNING state
StateType State = STOP; // Initialize state to STOP
StateType previousState = STOP; // Initialize previousState accordingly

// Helper function to find the end index for a value associated with a command character
int findValueEnd(const String& cmd, int startIndex) {
  int endIndex = cmd.length();
  for (int i = startIndex; i < cmd.length(); i++) {
    char c = cmd.charAt(i);
    // Stop if another command character is found
    if (c == 'G' || c == 'Y' || c == 'Z' || c == 'S' || c == 'P' || c == 'V' ||
        c == 'E' || c == 'T' || c == 'X' || c == 'D') {
      endIndex = i;
      break;
    }
  }
  return endIndex;
}


void setup() {
    Serial.begin(115200);
    Serial.println("Robot Odometry System Initialized");
    Serial.println("Commands:");
    Serial.println("  GX[val]Y[val]Z[val] - Go to position (cm, degrees)");
    Serial.println("  SETX[val]Y[val]Z[val] - Set current position (cm, degrees)");
    Serial.println("  V[val] - Set velocity");
    Serial.println("  S - Stop");
    Serial.println("  P - Request position");
    Serial.println("  D1 - Debug mode on");
    Serial.println("  D0 - Debug mode off");

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
    if (cmd.indexOf('S') != -1 && cmd.indexOf("SET") == -1) { // Stop command but not SET command
      State = STOP;
      if (debug) Serial.println("Command: STOP");
    } else if (cmd.indexOf('P') != -1) {
      previousState = State;
      State = SENDPOS;
      if (debug) Serial.println("Command: SEND POSITION");
    } else if (cmd.indexOf('D') != -1) {
      // Debug mode toggle
      int dPos = cmd.indexOf('D');
      int dEnd = findValueEnd(cmd, dPos + 1);
      int debugVal = cmd.substring(dPos + 1, dEnd).toInt();
      debug = (debugVal == 1);
      Serial.print("Debug mode: "); 
      Serial.println(debug ? "ON" : "OFF");
    } else if (State == STOP && (cmd.indexOf('G') != -1 || cmd.indexOf('V') != -1)) {
      // If stopped and a new goal or velocity is received, decide whether to align or navigate
      if (cmd.indexOf('G') != -1) { // If goal is present, start with alignment
           State = ALIGNING;
           if (debug) Serial.println("Command: ALIGN to new goal");
      } else { // If only velocity is present, just store it for future use
           if (debug) Serial.println("Command: Set velocity (robot remains stopped)");
      }
    }

    // Process SET command
    if (cmd.indexOf("SET") != -1) {
      int xPos = cmd.indexOf('X');
      int yPos = cmd.indexOf('Y');
      int zPos = cmd.indexOf('Z');
      
      if (xPos != -1 && yPos != -1 && zPos != -1 && xPos > 2 && yPos > xPos && zPos > yPos) {
        int xEnd = yPos;
        int yEnd = zPos;
        int zEnd = findValueEnd(cmd, zPos + 1);
        
        float new_x = cmd.substring(xPos + 1, xEnd).toFloat() / 100.0; // Convert from cm to meters
        float new_y = cmd.substring(yPos + 1, yEnd).toFloat() / 100.0; // Convert from cm to meters
        float new_theta = cmd.substring(zPos + 1, zEnd).toFloat() * M_PI / 180.0; // Convert degrees to rad
        
        robot.setPosition(new_x, new_y, new_theta);
        
        if (debug) {
          Serial.print("SET position to X:");
          Serial.print(new_x * 100.0);
          Serial.print(" Y:");
          Serial.print(new_y * 100.0);
          Serial.print(" Z:");
          Serial.println(new_theta * 180.0 / M_PI);
        }
        
        // Always send a position update after setting a new position
        Serial.print("POS,X:");
        Serial.print(robot.x * 100.0); // Convert x to cm
        Serial.print(",Y:");
        Serial.print(robot.y * 100.0); // Convert y to cm
        Serial.print(",Z:");
        Serial.println(robot.theta * 180.0 / M_PI); // Convert theta to degrees
      }
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
        // If a new goal is set, ensure we start the alignment process (unless S was also sent)
        if (State != STOP && State != SENDPOS) { // Avoid changing state if stopped or just sending pos
             State = ALIGNING; // Start with alignment
        }
        
        if (debug) {
          Serial.print("New goal: X:");
          Serial.print(x_goal * 100.0);
          Serial.print(" Y:");
          Serial.print(y_goal * 100.0);
          Serial.print(" Z:");
          Serial.println(theta_goal * 180.0 / M_PI);
        }
    }

    // Parse Velocity (V)
    if (vPos != -1) {
      int vEnd = findValueEnd(cmd, vPos + 1);
      v = cmd.substring(vPos + 1, vEnd).toFloat();
      
      if (debug) {
        Serial.print("Set velocity: ");
        Serial.println(v);
      }
    }
  }

  robot.updateOdometry(motorR.pos, motorL.pos);
  
  // Safety check for NaN in robot position
  if (isnan(robot.x) || isnan(robot.y) || isnan(robot.theta)) {
    if (debug) Serial.println("ERROR: NaN detected in position. Resetting to origin.");
    robot.setPosition(0, 0, 0);
  }

  switch (State) {
    case STOP:
      motorR.setMotorSpeed(0);
      motorL.setMotorSpeed(0);
      break;

    case ALIGNING:
      { // Use braces to create a local scope for variables
        float theta_diff = theta_goal - robot.theta;
        // Normalize angle difference to [-PI, PI]
        theta_diff = atan2(sin(theta_diff), cos(theta_diff));

        if (abs(theta_diff) < alignment_tolerance) {
          // Alignment complete, transition to NAVIGATION
          State = NAVIGATION;
          motorR.setMotorSpeed(0); // Stop briefly before navigating
          motorL.setMotorSpeed(0);
          if (debug) Serial.println("Alignment complete. Starting Navigation.");
        } else {
          // Still aligning, rotate in place (v=0)
          float current_v = 0; // Linear velocity is zero during alignment
          w = K_align * theta_diff; // Proportional control for angular velocity

          // Add saturation for angular velocity during alignment
          float max_align_w = 1.5; // rad/s - limit rotation speed
          w = constrain(w, -max_align_w, max_align_w);

          float speedR = (current_v + w * L) / r;
          float speedL = (current_v - w * L) / r;

          motorR.setMotorSpeed(speedR);
          motorL.setMotorSpeed(speedL);

          // Send position periodically during alignment if debug is on
          if (debug) {
            Serial.print("ALIGN: theta_diff=");
            Serial.print(theta_diff * 180.0 / M_PI);
            Serial.print(" w=");
            Serial.println(w);
          }
          
          // Always send position data during movement
          Serial.print("POS,X:");
          Serial.print(robot.x * 100.0); // Convert x to cm
          Serial.print(",Y:");
          Serial.print(robot.y * 100.0); // Convert y to cm
          Serial.print(",Z:");
          Serial.println(robot.theta * 180.0 / M_PI); // Convert theta to degrees
        }
      }
      break;

    case NAVIGATION:
      // Check if goal is reached before calculating new speeds
      if (sqrt(pow(robot.y - y_goal, 2) + pow(robot.x - x_goal, 2)) < eps) { // Use eps for goal check
          State = STOP;
          motorR.setMotorSpeed(0); // Stop motors immediately upon reaching goal
          motorL.setMotorSpeed(0);
          if (debug) Serial.println("Goal reached. Stopping.");
          
          // Always send final position when goal is reached
          Serial.print("POS,X:");
          Serial.print(robot.x * 100.0);
          Serial.print(",Y:");
          Serial.print(robot.y * 100.0);
          Serial.print(",Z:");
          Serial.print(robot.theta * 180.0 / M_PI);
          Serial.println(",GOAL_REACHED");
      } else {
          float thetaref = atan2(y_goal - robot.y, x_goal - robot.x);
          // Normalize angle difference to [-PI, PI]
          float angle_diff = atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
          w = K1 * angle_diff; // Use K1 for navigation angular control

          // Basic saturation for angular velocity
          float max_w = 2.0; // rad/s
          w = constrain(w, -max_w, max_w);

          // Use the velocity 'v' set by the 'V' command or the default
          // Add soft start - ramp up to desired velocity
          static float current_v = 0;
          if (current_v < v) {
            current_v += 0.2; // Increment for smooth acceleration
            if (current_v > v) current_v = v;
          } else if (current_v > v) {
            current_v -= 0.2; // Decrement for smooth deceleration
            if (current_v < v) current_v = v;
          }
          
          float speedR = (current_v + w * L) / r;
          float speedL = (current_v - w * L) / r;

          motorR.setMotorSpeed(speedR);
          motorL.setMotorSpeed(speedL);

          // Print debug info during navigation
          if (debug) {
            Serial.print("NAV: dist=");
            Serial.print(sqrt(pow(robot.y - y_goal, 2) + pow(robot.x - x_goal, 2)));
            Serial.print(" v=");
            Serial.print(current_v);
            Serial.print(" w=");
            Serial.println(w);
          }
          
          // Send position periodically during navigation
          static unsigned long lastPosUpdate = 0;
          if (millis() - lastPosUpdate > 100) { // Update position 10 times per second
            Serial.print("POS,X:");
            Serial.print(robot.x * 100.0); // Convert x to cm
            Serial.print(",Y:");
            Serial.print(robot.y * 100.0); // Convert y to cm
            Serial.print(",Z:");
            Serial.println(robot.theta * 180.0 / M_PI); // Convert theta to degrees
            lastPosUpdate = millis();
          }
      }
      break;

    case SENDPOS:
      Serial.print("POS,X:");
      Serial.print(robot.x * 100.0); // Convert x to cm
      Serial.print(",Y:");
      Serial.print(robot.y * 100.0); // Convert y to cm
      Serial.print(",Z:");
      Serial.println(robot.theta * 180.0 / M_PI); // Convert theta to degrees
      // Restore the state from before SENDPOS was triggered
      State = previousState;
      break;
  }
}