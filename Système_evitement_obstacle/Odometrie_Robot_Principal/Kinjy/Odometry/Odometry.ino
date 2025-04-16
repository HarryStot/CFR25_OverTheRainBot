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
float K1 = 120; // Tuning parameter for angular speed during navigation
float K_align = 10.0; // Tuning parameter for angular speed during alignment
float eps = 0.05; // Position tolerance
float alignment_tolerance = 0.05; // Angular tolerance for alignment (radians)

enum StateType { STOP, ALIGNING, NAVIGATION, SENDPOS }; // Added ALIGNING state
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
        // If stopped and a new goal or velocity is received, decide whether to align or navigate
        if (cmd.indexOf('G') != -1) { // If goal is present, start with alignment
             State = ALIGNING;
        } else { // If only velocity is present, go directly to navigation (or handle as error/ignore?)
             // For now, let's assume V without G means just set speed for potential future NAV
             // State = NAVIGATION; // Or keep STOP until G is received? Let's keep STOP.
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
    }

    // Parse Velocity (V)
    if (vPos != -1) {
      int vEnd = findValueEnd(cmd, vPos + 1);
      v = cmd.substring(vPos + 1, vEnd).toFloat();
       // Setting velocity doesn't automatically start navigation anymore,
       // it just sets the speed for the NAVIGATION phase.
       // Remove state change here:
       // if (State != STOP) {
       //     State = NAVIGATION;
       // }
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
          // Serial.println("Alignment complete. Starting Navigation."); // Debug message
        } else {
          // Still aligning, rotate in place (v=0)
          float current_v = 0; // Linear velocity is zero during alignment
          w = K_align * theta_diff; // Proportional control for angular velocity

          // Optional: Add saturation for angular velocity during alignment
          // float max_align_w = 1.5; // rad/s
          // w = constrain(w, -max_align_w, max_align_w);

          float speedR = (current_v + w * L) / r;
          float speedL = (current_v - w * L) / r;

          motorR.setMotorSpeed(speedR);
          motorL.setMotorSpeed(speedL);

          // Send position periodically during alignment
          Serial.print("ALIGN"); Serial.print(",X:"); // Indicate alignment phase
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
          // Serial.println("Goal reached. Stopping."); // Debug message
      } else {
          float thetaref = atan2(y_goal - robot.y, x_goal - robot.x);
          // Normalize angle difference to [-PI, PI]
          float angle_diff = atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
          w = K1 * angle_diff; // Use K1 for navigation angular control

          // Basic saturation for angular velocity (optional, adjust limits as needed)
          // float max_w = 2.0; // rad/s
          // w = constrain(w, -max_w, max_w);

          // Use the velocity 'v' set by the 'V' command or the default
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