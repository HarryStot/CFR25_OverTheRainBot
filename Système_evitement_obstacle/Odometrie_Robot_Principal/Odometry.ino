#include "Motor_Control.h"
#include "Odometry.h"

#define ENCAD 3  //Encodeur A du moteur droit
#define ENCBD 9  //Encodeur B du moteur droit
#define IN1D 11  //Tourner moteur en avant
#define IN2D 10  //Tourner moteur en arrière

#define ENCAG 2  //Encodeur A du moteur gauche
#define ENCBG 8  //Encodeur B du moteur gauche
#define IN1G  5  //Tourner moteur en avant
#define IN2G  6  //Tourner moteur en arrière

Motor motorR(ENCAD, ENCBD, IN1D, IN2D);  //Crée un objet moteur
Motor motorL(ENCAG, ENCBG, IN1G, IN2G);

// Caractéristiques robot + Odométrie
float phiR = 0.0, phiL = 0.0;
float L = 0.095 / 2;  //Distance entre les roues sur 2
float r = 0.08 / 2;   //Rayon des roues
float v, w;
float posL = 0, posR = 0;
float thetaref;
float eps = 0.1;  //Précision acceptable sur la position. Si eps est trop petit il est possible que le robot n'atteigne jamais sa position.
float x_int, y_int;
float pi = 3.14;
float chrono;

int iter = 0;
int State = 0;

float K1 = 30;  //Correcteur proportionnel à déterminer

//Objectifs
float ly_goal[4] = { 0, 1, 1 };
float lx_goal[4] = { 0, 0, 2 };
int arrayLength = sizeof(lx_goal) / sizeof(lx_goal[0]);  //La fonction length n'existe pas sur Arduino
float x_goal, y_goal;
int i_goal = 0;

Odometry robot(L, r);

//Capteurs
float dist = 99999;

void setup() {
  Serial.begin(9600);

  motorR.init();                                                        //Initialisation du moteur
  attachInterrupt(digitalPinToInterrupt(ENCAD), readEncoderD, RISING);  //Les encodeurs doivent être branchées sur des pins d'interruption. Sur Arduino Méga se sont les pins 2,3,18,19,"STOP" et 21

  motorL.init();
  attachInterrupt(digitalPinToInterrupt(ENCAG), readEncoderG, RISING);

  delay(1500);
  chrono = millis();
}

void loop() {

  posR = -motorR.pos;  //Position des roues
  posL = motorL.pos;
  robot.updateOdometry(posR, posL, x_goal, y_goal);  //Calcul de l'odométrie rangé dans Odometry.h et Odometry.cpp pour gagner de la place

  //Machine d'états
  if (State == "Depart")
  {
    motorR.setMotorSpeed(0);  //Le control des moteurs se fait avec cette fonction, elle prend en argument une vitesse de rotation phi
    motorL.setMotorSpeed(0);
  }

  if (State == "Navigation")
  {
    x_goal = lx_goal[i_goal];
    y_goal = ly_goal[i_goal];
    v = 10;  //A régler selon l'étalonnage et la saturation des moteurs
    thetaref = atan2((y_goal - robot.y), (x_goal - robot.x));
    w = K1 * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
    phiR = (v + w * L) / r;
    phiL = (v - w * L) / r;

    // Serial.print("phiR = ");
    // Serial.print(phiR);
    // Serial.print("   ");
    // Serial.print("phiL = ");
    // Serial.println(phiL);

    motorR.setMotorSpeed(phiR);
    motorL.setMotorSpeed(phiL);
    x_int = robot.x + dist * 0.01 * cos(robot.theta) - 0.3 * sin(robot.theta);  //Utile pour l'évitement d'obstacle
    y_int = robot.y + dist * 0.01 * sin(robot.theta) + 0.3 * cos(robot.theta);  //Il sert à calculer un nouveau point à côté de l'obstacle
  }

  if (State == "Evitement")  //Évitement d'obstacle (méthode de Kinjy, Lilian et Guillaume en robotique mais à modifier par la team 4 plus tard)
  {
    thetaref = atan2((y_int - robot.y), (x_int - robot.x));
    w = K1 * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
    phiR = (v + w * L) / r;
    phiL = (v - w * L) / r;

    motorR.setMotorSpeed(phiR);
    motorL.setMotorSpeed(phiL);
    Serial.println(absf(robot.x - x_int));
    Serial.println(absf(robot.y - y_int));
  }

  if (State == "STOP")
  {
    Serial.println("STOP");
    motorR.setMotorSpeed(0);
    motorL.setMotorSpeed(0);
  }

  //Transitions
  if (State == "Depart" && dist > 30) { State = "Navigation"; }
  if (State == "Navigation" && (absf(robot.y - y_goal) < eps) && (absf(robot.x - x_goal)) < eps && i_goal != arrayLength) { State = "Navigation"; i_goal = i_goal + 1;}                                                                                                                             //Utiliser absf pour calculer une valeur absolue
  if (State == "Navigation" && (absf(robot.y - y_goal) < eps) && (absf(robot.x - x_goal)) < eps && i_goal == arrayLength) { State = "STOP"; }  //Jsp pourquoi la fct abs de Arduino ne marche pas correctement
  if (State == "Navigation" && dist < 50) { State = "Evitement"; }
  if (State == "Evitement" && (absf(robot.y - y_int) < eps) && (absf(robot.x - x_int)) < eps) { State = "Navigation"; }
  if (millis() - chrono > 85000) { State = "Navigation"; lx_goal[4] = {0}; ly_goal[4] = {0};}
  if (millis() - chrono > 99000) { State = "STOP"; }
  iter = iter + 1;
}

float absf(float val) {
  return val < 0 ? -val : val;
}

void readEncoderD() {  //Tout est dans le nom
  motorR.readEncoder();
}
void readEncoderG() {
  motorL.readEncoder();
}