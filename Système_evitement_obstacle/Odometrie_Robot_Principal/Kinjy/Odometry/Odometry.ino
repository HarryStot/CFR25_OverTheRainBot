#include "Motor_Control.h"
#include "Odometry.h"

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
float pi = 3.14;

Motor motorR(ENCAD, ENCBD, PWMD, DIRD);
Motor motorL(ENCAG, ENCBG, PWMG, DIRG);
Odometry robot(L, r);

float v = 10.0, w;
float x_goal = 0, y_goal = 1, theta_goal = pi/2;
float K1 = 120;
float eps = 0.1;

enum StateType { STOP, NAVIGATION, SENDPOS };
StateType State = NAVIGATION;

void setup() {
    Serial.begin(9600);

    motorR.init();
    motorL.init();
    attachInterrupt(digitalPinToInterrupt(ENCAD), [] { motorR.readEncoder(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCAG), [] { motorL.readEncoder(); }, RISING);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    int g = cmd.indexOf('G');
    int y = cmd.indexOf('Y');
    int z = cmd.indexOf('Z');
    int vPos = cmd.indexOf('V');

    if (g != -1 && y != -1 && z != -1)
      x_goal = cmd.substring(g + 1, y).toFloat(),
      y_goal = cmd.substring(y + 1, z).toFloat(),
      theta_goal = cmd.substring(z + 1).toFloat();

    if (cmd.indexOf('S') != -1) {
      State = STOP;
    }
    if (cmd.indexOf('P') != -1) {
      State = SENDPOS;
    }

    // Extraire la valeur après V
    if (vPos != -1) {
      int end = cmd.length();
      for (int i = vPos + 1; i < cmd.length(); i++) {
        char c = cmd.charAt(i);
        if (c == 'G' || c == 'Y' || c == 'Z' || c == 'S' || c == 'P' || c == 'V') {
          end = i;
          break;
        }
      }
      v = cmd.substring(vPos + 1, end).toFloat();
    }
  }

  robot.updateOdometry(motorR.pos, motorL.pos);

  switch (State) {
    case STOP:
      motorR.setMotorSpeed(0);
      motorL.setMotorSpeed(0);
      break;
    
    case NAVIGATION:
      float thetaref = atan2(y_goal - robot.y, x_goal - robot.x);
      w = K1 * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));

      float speedR = (v + w * L) / r;
      float speedL = (v - w * L) / r;

      motorR.setMotorSpeed(speedR);
      motorL.setMotorSpeed(speedL);

      Serial.print("POS"); Serial.print(",X:");
      Serial.print(robot.x); Serial.print(",Y:");
      Serial.print(robot.y); Serial.print(",Z:");
      Serial.println(robot.theta);
      break;
  
    case SENDPOS: // Je n'ai aucune idée de pourquoi ce qui est dedans ne s'éxecute pas
      Serial.print("sendPOS"); Serial.print(",X:");
      Serial.print(robot.x); Serial.print(",Y:");
      Serial.print(robot.y); Serial.print(",Z:");
      Serial.println(robot.theta);
      State = NAVIGATION; // Remettre l'état à NAVIGATION après l'envoi
      break;
  }
  if (State == NAVIGATION && sqrt(pow(robot.y - y_goal, 2) + pow(robot.x - x_goal, 2)) < 0.05) { State = STOP; }
  // Serial.println(State);
}