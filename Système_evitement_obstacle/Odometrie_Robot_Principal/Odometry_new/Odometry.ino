#include "Motor_Control.h"
#include "Odometry.h"

#define ENCAD 19
#define ENCBD 18
#define PWMD 3
#define DIRD 12
#define ENCAG 21
#define ENCBG 20
#define PWMG 11
#define DIRG 13

String us_reception = "";

const uint8_t pinTirette = 27;

float L = 0.185 / 2;
float r = 0.084 / 2;
float pi = 3.14;

Motor motorR(ENCAD, ENCBD, PWMD, DIRD);
Motor motorL(ENCAG, ENCBG, PWMG, DIRG);
Odometry robot(L, r);

float v = 7.0, w;
float speedR = 0, speedL = 0;
float angle_error = pi, thetaref;
float x_goal = 0, y_goal = 0, theta_goal = 0;
float K1 = 80;
float eps = 0.05, eps_theta = 0.1;

enum StateType { STOP, NAVIGATION, ORIENTATION, SENDPOS };
StateType State = NAVIGATION;

void setup() {
    Serial.begin(9600);

    pinMode(pinTirette, INPUT_PULLUP);
    //wait(0);
    Serial.println("Initialisation");
    //wait(1);
    Serial.println("Match");

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
      State = NAVIGATION;

    if (cmd.indexOf('S') != -1) {
      State = STOP;
    }
    if (cmd.indexOf('P') != -1) {
      State = SENDPOS;
    }

    if (cmd.startsWith("us")) {
      State = STOP;
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

  robot.updateOdometry(-motorR.pos, motorL.pos);
  
  switch (State) {
    case STOP:
      motorR.setMotorSpeed(0);
      motorL.setMotorSpeed(0);
      Serial.println("STOP");
      break;
    
    case NAVIGATION:
      thetaref = atan2(y_goal - robot.y, x_goal - robot.x);
      w = K1 * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));

      speedR = (v + w * L) / r;
      speedL = (v - w * L) / r;

      motorR.setMotorSpeed(speedR);
      motorL.setMotorSpeed(speedL);

      Serial.print("POS"); Serial.print(",X:");
      Serial.print(robot.x); Serial.print(",Y:");
      Serial.print(robot.y); Serial.print(",Z:");
      Serial.println(robot.theta);
      break;

    case ORIENTATION:
      angle_error = atan2(sin(theta_goal - robot.theta), cos(theta_goal - robot.theta));
      w = K1 * atan2(sin(theta_goal - robot.theta), cos(theta_goal - robot.theta));

      // Ici v = 0
      speedR = w * L / r;
      speedL = -w * L / r;
      Serial.print(speedR);
      Serial.print(" ");
      Serial.println(speedL);
      motorR.setMotorSpeed(speedR);
      motorL.setMotorSpeed(speedL);
      break;

    case SENDPOS:
      Serial.print("sendPOS"); Serial.print(",X:");
      Serial.print(robot.x); Serial.print(",Y:");
      Serial.print(robot.y); Serial.print(",Z:");
      Serial.println(robot.theta);
      break;
  }
  //if (State == NAVIGATION && sqrt(pow(robot.y - y_goal, 2) + pow(robot.x - x_goal, 2)) < eps && fabs(robot.theta - theta_goal) < eps_theta) { State = STOP; }
  if (State == NAVIGATION && sqrt(pow(robot.y - y_goal, 2) + pow(robot.x - x_goal, 2)) < eps) { State = ORIENTATION; }
  if (State == ORIENTATION && fabs(angle_error) < eps_theta) { State = STOP; }                                         
  Serial.println(State);
}

void wait(int status) { //Attends que la tirette soit rétirée
  while (digitalRead(pinTirette) != status) {
    delay(100);
  }
}
