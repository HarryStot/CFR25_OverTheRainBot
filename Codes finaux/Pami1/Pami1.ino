#include "Motor_Control.h"
#include "Odometry.h"
#include "Ultrason.h"

// --- Définition des broches pour les moteurs et encodeurs ---
#define ENCAD 3
#define ENCBD 12
#define PWMD 9
#define PINAD 6
#define PINBD 7

#define ENCAG 2
#define ENCBG 11
#define PWMG 10
#define PINAG 4
#define PINBG 5

#define TRIG_PIN A0
#define ECHO_PIN A1

#define DANSEUR 8 //A definir
#define TIRETTE A5

// --- Dimensions du robot ---
const float L = 0.096 / 2;
const float r = 0.04;
const float pi = 3.1416;

// --- Objets moteurs et odométrie ---
Motor motorR(ENCAD, ENCBD, PWMD, PINAD, PINBD);
Motor motorL(ENCAG, ENCBG, PWMG, PINAG, PINBG);
Odometry robot(L, r);
Ultrason capteurUltrason(TRIG_PIN, ECHO_PIN);

// --- Position initiale et objectifs ---
float x_init = 0, y_init = 0, theta_init = 0;
float thetaref = 0;
int x = 0, y = 0, z = 0;

struct Goal {
    float x;
    float y;
    float theta;
};

Goal goals[1] = {
    {1.0, -0.38, 1.57}
};

int goalCount = 1;
int currentGoalIndex = 0;
float x_goal = goals[currentGoalIndex].x, y_goal = goals[currentGoalIndex].y, theta_goal = goals[currentGoalIndex].theta;

// --- Gains pour la navigation et l'orientation ---
const float initial_Kp1 = 1, initial_Kp2 = 40;
float Kp1 = initial_Kp1;  // NAVIGATION
float K = 60, Kp2 = initial_Kp2;  // ORIENTATION
const float eps = 0.02, eps_theta = 0.08;

// --- Variables pour l'ajustement dynamique de Kp2 ---
float last_x = 0, last_y = 0, last_theta = 0;
int immobile_counter = 0;
const int immobile_threshold1 = 3, immobile_threshold2 = 2;
const float Kp1_increment = 0.2, Kp2_increment = 4;
const float max_Kp1 = 9999, max_Kp2 = 9999; // Je sais pas vraiment si il faut mettre un max j'ai pas encore eu de problème sans

// --- Vitesse et contrôle temporel ---
float v = 2.0, w = 0; //v = 1.5 pour robot 2
float speedR = 0, speedL = 0;
float angle_error = pi;
unsigned long lastUpdateTime = 0;
float dt = 0.0;

// --- États du robot ---
enum StateType { STOP, NAVIGATION, ORIENTATION, OBSTACLE, DANSER};
StateType State = NAVIGATION;
StateType PreviousState = NAVIGATION;

// --- Autres variables ---
int test = 0;

void setup() {
    Serial.begin(115200);
    pinMode(TIRETTE, INPUT_PULLUP);
    Serial.println("Prêt");
    wait(0);
    Serial.println("Initialisation");
    wait(1);
    Serial.println("Match");

    motorR.init();
    motorL.init();
    attachInterrupt(digitalPinToInterrupt(ENCAD), [] { motorR.readEncoder(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCAG), [] { motorL.readEncoder(); }, RISING);
    pinMode(DANSEUR, OUTPUT);
    digitalWrite(DANSEUR, LOW); 
    robot.init(x_init, y_init, theta_init);
    delay(1000);
}

unsigned long previousMillis = 0;
const unsigned long interval = 500;
bool parti = false;

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  }
  
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }

  robot.updateOdometry(motorR.pos, motorL.pos);

  float distance = capteurUltrason.readDistanceCM();
  Serial.println(distance);
  if (State != OBSTACLE && distance > 0 && distance < 10.0) {
      PreviousState = State;
      State = OBSTACLE;
      Serial.println("Obstacle détecté !");
  }
  else if (State == OBSTACLE && distance >= 30.0) {
      State = PreviousState;
      Serial.println("Obstacle dégagé. Reprise.");
  }

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
      break;

    case OBSTACLE:
      Freiner();
      Serial.println("En attente que l'obstacle se dégage...");
      break;

    case DANSER:
      digitalWrite(DANSEUR, HIGH);
      break;
  }
  // Serial.println(currentMillis);
  //Transitions
  //DEPART A 85s
  if (currentMillis >= 85000 && !parti) {State = NAVIGATION; parti = true;}
  if (State == NAVIGATION  && sqrt(pow(robot.y - y_goal, 2) + pow(robot.x - x_goal, 2)) < eps) { Freiner(); Kp2 = initial_Kp2; Kp1 = initial_Kp1; angle_error = atan2(sin(theta_goal - robot.theta), cos(theta_goal - robot.theta)); State = ORIENTATION; }
  if (State == ORIENTATION && fabs(angle_error) < eps_theta && currentGoalIndex < goalCount) { Freiner(); Kp2 = initial_Kp2; Kp1 = initial_Kp1; currentGoalIndex++; x_goal = goals[currentGoalIndex].x; y_goal = goals[currentGoalIndex].y; theta_goal = goals[currentGoalIndex].theta; State = NAVIGATION; }        
  if (currentGoalIndex == goalCount) { State = DANSER; }                    
}

void Freiner() {
  motorR.setMotorSpeed(0);
  motorL.setMotorSpeed(0);
}

void setNavigationSpeed(float v, float w) {
    float speedR = (v + w * L) / r;
    float speedL = (v - w * L) / r;

    motorR.setMotorSpeed(speedR);
    motorL.setMotorSpeed(speedL);
}

void setOrientationSpeed(float w) {
    const int MAX_SPEED = 80;
    const int MIN_SPEED = 20;

    float speedR = (w * L) / r;
    float speedL = (-w * L) / r;

    // Évite appel à constrain() en remplaçant par logique directe
    if (speedR > MAX_SPEED) speedR = MAX_SPEED;
    else if (speedR < -MAX_SPEED) speedR = -MAX_SPEED;

    if (speedL > MAX_SPEED) speedL = MAX_SPEED;
    else if (speedL < -MAX_SPEED) speedL = -MAX_SPEED;

    // Appliquer un minimum pour forcer un démarrage moteur, mais éviter appel à max()/min()
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
            // Serial.print("Kp1 ajusté : ");
            // Serial.println(Kp1);
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
              State = NAVIGATION;
          } else if (cmd.startsWith("INIT")) {
              x_init = xVal;
              y_init = yVal;
              theta_init = zVal;
              robot.init(x_init, y_init, theta_init);
              Serial.println("Initialisation : ");
          }

          // Affichage pour vérification
          Serial.print("X : "); Serial.println(xVal);
          Serial.print("Y : "); Serial.println(yVal);
          Serial.print("Theta : "); Serial.println(zVal);
      }
  }

  // Gestion des commandes simples
  if (cmd.startsWith("S")) State = STOP;

  // Gestion de la vitesse
  int vPos = cmd.indexOf('V');
  if (vPos != -1) {
      v = cmd.substring(vPos + 1).toFloat();
      Serial.print("Vitesse : "); Serial.println(v);
  }
}

void wait(int status) {
  while (digitalRead(TIRETTE) != status) {
    delay(100);
  }
}
