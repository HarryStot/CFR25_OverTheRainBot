#include "Motor_Control.h"
#include "Odometry.h"

// --- Définition des broches pour les moteurs et encodeurs ---
#define ENCAD 19
#define ENCBD 18
#define PWMD 3
#define DIRD 12
#define BRKD 9

#define ENCAG 21
#define ENCBG 20
#define PWMG 11
#define DIRG 13
#define BRKG 8

// --- Dimensions du robot ---
const float L = 0.325 / 2;
const float r = 0.084 / 2;
const float pi = 3.1416;

// --- Objets moteurs et odométrie ---
Motor motorR(ENCAD, ENCBD, PWMD, DIRD, BRKD, BRKG);
Motor motorL(ENCAG, ENCBG, PWMG, DIRG, BRKD, BRKG);
Odometry robot(L, r);

// --- Position initiale et objectifs ---
float x_init = 0, y_init = 0, theta_init = 0;
float x_goal = 0, y_goal = 0, theta_goal = 0;
float thetaref = 0;
int x = 0, y = 0, z = 0;

// --- Gains pour la navigation et l'orientation ---
const float initial_Kp1 = 1, initial_Kp2 = 40;
float K = 80, Kp1 = initial_Kp1;  // NAVIGATION
float Kp2 = initial_Kp2;  // ORIENTATION
const float eps = 0.04, eps_theta = 0.04;

// --- Variables pour l'ajustement dynamique de Kp2 ---
float last_x = 0, last_y = 0, last_theta = 0;
int immobile_counter = 0;
const int immobile_threshold1 = 5, immobile_threshold2 = 2;
const float Kp1_increment = 0.1, Kp2_increment = 4;
const float max_Kp1 = 9999, max_Kp2 = 9999; // Je sais pas vraiment si il faut mettre un max j'ai pas encore eu de problème sans

// --- Vitesse et contrôle temporel ---
float v = 10.0, w = 0;
float speedR = 0, speedL = 0;
float angle_error = pi;
unsigned long lastUpdateTime = 0;
float dt = 0.0;

// --- États du robot ---
enum StateType { STOP, NAVIGATION, ORIENTATION, SENDPOS, AVANCER, RECULER};
StateType State = STOP;

// --- Autres variables ---
int test = 0;
float last_cmd_x = NAN, last_cmd_y = NAN;

void setup() {
    Serial.begin(115200);
    delay(1000);
    motorR.init();
    motorL.init();
    attachInterrupt(digitalPinToInterrupt(ENCAD), [] { motorR.readEncoder(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCAG), [] { motorL.readEncoder(); }, RISING);

    robot.init(x_init, y_init, theta_init);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }

  robot.updateOdometry(-motorR.pos, motorL.pos);
  
  switch (State) {
    case STOP:
      Freiner();
      Serial.print("POS,X:"); Serial.print(robot.x); 
      Serial.print(",Y:"); Serial.print(robot.y); 
      Serial.print(",Z:"); Serial.println(robot.theta);
      // Serial.print(angle_error); Serial.print("   ");Serial.print(speedR);Serial.print("   ");Serial.println(speedL); 
      break;
    
    case NAVIGATION:
      adjustKp1();

      thetaref = atan2(y_goal - robot.y, x_goal - robot.x);
      w = K * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
      setNavigationSpeed(Kp1*v, w);
      
      Serial.print("POS,X:"); Serial.print(robot.x); 
      Serial.print(",Y:"); Serial.print(robot.y); 
      Serial.print(",Z:"); Serial.println(robot.theta);
      // Serial.print(angle_error); Serial.print("   ");Serial.print(speedR);Serial.print("   ");Serial.println(speedL); 
      break;

    case ORIENTATION:
      // Vérification de l'immobilité pour ajuster Kp2 (Utile si les moteurs n'arrivent pas à tourner et qu'il y a plus de frictions que dans les phases de réglage)
      adjustKp2();

      angle_error = atan2(sin(theta_goal - robot.theta), cos(theta_goal - robot.theta));
      w = Kp2 * angle_error;
      setOrientationSpeed(w);

      Serial.print("POS,X:"); Serial.print(robot.x); 
      Serial.print(",Y:"); Serial.print(robot.y); 
      Serial.print(",Z:"); Serial.println(robot.theta);
      // Serial.print(angle_error); Serial.print("   ");Serial.print(speedR);Serial.print("   ");Serial.println(speedL); 
      Serial.print("Kp2 ajusté : ");
      Serial.println(Kp2);
      break;

    case SENDPOS:
      Serial.print("POS"); Serial.print(",X:");
      Serial.print(robot.x); Serial.print(",Y:");
      Serial.print(robot.y); Serial.print(",Z:");
      Serial.println(robot.theta);
      break;

    case AVANCER:
      adjustKp1();
    
      thetaref = atan2(y_goal - robot.y, x_goal - robot.x);  // direction naturelle vers l'avant
      thetaref = atan2(sin(thetaref), cos(thetaref));        // normalise entre -π et π
      w = K * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
      setNavigationSpeed(Kp1*v, w);

      Serial.print("POS"); Serial.print(",X:");
      Serial.print(robot.x); Serial.print(",Y:");
      Serial.print(robot.y); Serial.print(",Z:");
      Serial.println(robot.theta);
      break;

    case RECULER:
      adjustKp1();

      thetaref = atan2(y_goal - robot.y, x_goal - robot.x) + pi;
      thetaref = atan2(sin(thetaref), cos(thetaref));  // Normalise l'angle entre -π et π
      w = K * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
      setNavigationSpeed(-Kp1*v, w);

      Serial.print("POS"); Serial.print(",X:");
      Serial.print(robot.x); Serial.print(",Y:");
      Serial.print(robot.y); Serial.print(",Z:");
      Serial.println(robot.theta);
      break;
  }
  // Serial.println(State);
  if ((State == NAVIGATION || State == RECULER || State == AVANCER) && sqrt(pow(robot.y - y_goal, 2) + pow(robot.x - x_goal, 2)) < eps) {Freiner(); Serial.println("STOP_NAVIGATION"); Kp1 = initial_Kp1; Kp2 = initial_Kp2; angle_error = atan2(sin(theta_goal - robot.theta), cos(theta_goal - robot.theta)); State = ORIENTATION; }
  if (State == ORIENTATION && fabs(angle_error) < eps_theta) {Freiner(); Serial.println("STOP_ORIENTATION"); Kp1 = initial_Kp1; Kp2 = initial_Kp2; State = STOP;}   
                              
}

void Freiner() {
  digitalWrite(BRKD, HIGH); // Activation du frein
  digitalWrite(BRKG, HIGH);
  analogWrite(PWMD, 255); // Coupe le signal PWM en court-circuitant
  analogWrite(PWMG, 255);
  delay(400);
}

void setNavigationSpeed(float v, float w) {
    float speedR = (v + w * L) / r;
    float speedL = (v - w * L) / r;
    Serial.println(speedR);
    Serial.println(speedL);
    motorR.setMotorSpeed(speedR);
    motorL.setMotorSpeed(speedL);
}

void setOrientationSpeed(float w) {
    const int MIN_SPEED = 80;

    float speedR = (w * L) / r;
    float speedL = (-w * L) / r;

    if (speedR > 0 && speedR < MIN_SPEED) speedR = MIN_SPEED;
    else if (speedR < 0 && speedR > -MIN_SPEED) speedR = -MIN_SPEED;

    if (speedL > 0 && speedL < MIN_SPEED) speedL = MIN_SPEED;
    else if (speedL < 0 && speedL > -MIN_SPEED) speedL = -MIN_SPEED;

    motorR.setMotorSpeed(speedR);
    motorL.setMotorSpeed(speedL);
}

void adjustKp1() {
    // Vérifie si le robot est immobile
    if (robot.x == last_x && robot.y == last_y && robot.theta == last_theta) {
        immobile_counter++;
        if (immobile_counter >= immobile_threshold1) {
            Kp1 = min(Kp1 + Kp1_increment, max_Kp1);
            immobile_counter = 0;
            Serial.print("Kp1 ajusté : ");
            Serial.println(Kp1);
        }
    } else {
        immobile_counter = 0;
    }

    // Mise à jour des dernières positions
    last_x = robot.x;
    last_y = robot.y;
    last_theta = robot.theta;
}

void adjustKp2() {
    // Vérifie si le robot est immobile
    if (robot.x == last_x && robot.y == last_y && robot.theta == last_theta) {
        immobile_counter++;
        if (immobile_counter >= immobile_threshold2) {
            Kp2 = min(Kp2 + Kp2_increment, max_Kp2);
            immobile_counter = 0;
            // Serial.print("Kp2 ajusté : ");
            // Serial.println(Kp2);
        }
    } else {
        immobile_counter = 0;
    }

    // Mise à jour des dernières positions
    last_x = robot.x;
    last_y = robot.y;
    last_theta = robot.theta;
}

void parseCommand(String cmd) {
  cmd.trim();

  // Gestion des positions G et INIT
  if (cmd.startsWith("G") || cmd.startsWith("INIT")) {
      int x = cmd.indexOf('X');
      int y = cmd.indexOf('Y');
      int z = cmd.indexOf('Z');
      
      if (x != -1 && y != -1 && z != -1 && x < y && y < z) {
          float xVal = cmd.substring(x + 1, y).toFloat();
          float yVal = cmd.substring(y + 1, z).toFloat();
          float zVal = cmd.substring(z + 1).toFloat();

          if (cmd.startsWith("G")) {
              x_goal = xVal;
              y_goal = yVal;
              theta_goal = zVal;
              // Comparaison avec ancienne commande
              bool same_cmd = (x_goal == last_cmd_x && y_goal == last_cmd_y);

              last_cmd_x = x_goal;
              last_cmd_y = y_goal;

              // Si la position n’a pas changé, on va direct en ORIENTATION
              if (same_cmd) {
                Serial.println("STOP_NAVIGATION");
                State = ORIENTATION;
              } else {
                State = NAVIGATION;
              }

          } else if (cmd.startsWith("INIT")) {
              x_init = xVal;
              y_init = yVal;
              theta_init = zVal;
              robot.init(x_init, y_init, theta_init);
              // Serial.println("Initialisation : ");
          }

          // Affichage pour vérification
          Serial.print("POS"); Serial.print(",X:");
          Serial.print(x_init); Serial.print(",Y:");
          Serial.print(y_init); Serial.print(",Z:");
          Serial.println(theta_init);
      }
  }

  // Gestion des commandes simples
  if (cmd.startsWith("S")) State = STOP;
  if (cmd.startsWith("P")) State = SENDPOS;

  // Gestion de la vitesse
  int vPos = cmd.indexOf('V');
  if (vPos != -1) {
      v = cmd.substring(vPos + 1).toFloat();
      Serial.print("Vitesse : "); Serial.println(v);
  }

  // Gestion du forward
  if (cmd.startsWith("F")) {
    float dist = cmd.substring(1).toFloat();
    // Calcul du goal en marche avant
    x_goal = robot.x + dist * cos(robot.theta);
    y_goal = robot.y + dist * sin(robot.theta);
    theta_goal = robot.theta;
    theta_goal = atan2(sin(theta_goal), cos(theta_goal));

    // Serial.print(x_goal);Serial.print(" ");Serial.print(y_goal);Serial.print(" ");Serial.println(theta_goal);
    State = AVANCER;
  }

  // Gestion du recul
  if (cmd.startsWith("R")) {
      float dist = cmd.substring(1).toFloat();
      // Calcul du goal en marche arrière
      x_goal = robot.x - dist * cos(robot.theta);
      y_goal = robot.y - dist * sin(robot.theta);
      theta_goal = robot.theta;
      theta_goal = atan2(sin(theta_goal), cos(theta_goal));

      // Serial.print(x_goal);Serial.print(" ");Serial.print(y_goal);Serial.print(" ");Serial.println(theta_goal);
      State = RECULER;
  }
}
