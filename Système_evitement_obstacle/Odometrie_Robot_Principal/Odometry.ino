#include "Motor_Control.h"
#include "Odometry.h"
#include <stdint.h>
#include <VARSTEP_ultrasonic.h>

#define ENCAD 19  //Encodeur A du moteur droit
#define ENCBD 18  //Encodeur B du moteur droit
#define PWMD 11   //Control moteur droit
#define DIRD 13   //Gérer le sens de rotation du moteur droit

#define ENCAG 21  //Encodeur A du moteur gauche
#define ENCBG 20  //Encodeur B du moteur gauche
#define PWMG 3    //Control moteur gauche
#define DIRG 12   //Gérer le sens de rotation du moteur gauche

#define trigger_pin 8 //capteur
#define echo_pin 9 //capteur

Motor motorR(ENCAD, ENCBD, PWMD, DIRD);  //Crée un objet moteur
Motor motorL(ENCAG, ENCBG, PWMG, DIRG);

// Caractéristiques robot + Odométrie
float phiR = 0.0, phiL = 0.0;
float L = 0.245 / 2;  //Distance entre les roues sur 2
float r = 0.065 / 2;        //Rayon des roues
float v, w;
float posL = 0, posR = 0;
float thetaref;
float eps = 0.1;  //Précision acceptable sur la position. Si eps est trop petit il est possible que le robot n'atteigne jamais sa position.
float x_int, y_int;
float pi = 3.14;

int iter = 0; 
int State = 0;

const uint8_t pinTirette = 27; //Tirette

double dist; //Capteur ultrason 

float K1 = 30;  //Correcteur proportionnel à déterminer

//Objectifs
float ly_goal[4] = { 0, 1, 1, 2 };
float lx_goal[4] = { 0, 0, 2, 1 };
int arrayLength = sizeof(lx_goal) / sizeof(lx_goal[0]);  //La fonction length n'existe pas sur Arduino
float x_goal, y_goal;
int i_goal = 0;

Odometry robot(L, r);

VARSTEP_ultrasonic my_HCSR04(trigger_pin, echo_pin); 

void setup() {
  Serial.begin(9600);

  pinMode(pinTirette,INPUT_PULLUP );

  //Le contrôle de la tirette se fait avec le fonction "wait", elle prend en argument un état: 0 (tirette enfoncée) ou 1 (tirette enlévée)

  wait(0); // attend que la tirette soit enfoncée dans le support
  Serial.println("Initialisation");

  wait(1); // on attend que la tirette soit enlevée du support
  Serial.println("Match");

  motorR.init();                                                        //Initialisation du moteur
  attachInterrupt(digitalPinToInterrupt(ENCAD), readEncoderD, RISING);  //Les encodeurs doivent être branchées sur des pins d'interruption. Sur Arduino Méga se sont les pins 2,3,18,19,20 et 21

  motorL.init();
  attachInterrupt(digitalPinToInterrupt(ENCAG), readEncoderG, RISING);

  delay(1500);
}

void loop() {

  dist = my_HCSR04.distance_cm();
  posR = -motorR.pos;  //Position des roues
  posL = motorL.pos;
  robot.updateOdometry(posR, posL, x_goal, y_goal);  //Calcul de l'odométrie rangé dans Odometry.h et Odometry.cpp pour gagner de la place

  //Machine d'états
  if (State == 0)  //Départ
  {
    motorR.setMotorSpeed(0);  //Le controle des moteurs se fait avec cette fonction, elle prend en argument une vitesse de rotation phi
    motorL.setMotorSpeed(0);
  }

  if (State == 4)  //Navigation
  {
    x_goal = lx_goal[i_goal];
    y_goal = ly_goal[i_goal];
    v = 10;  //A régler selon l'étalonnage et la saturation des moteurs
    thetaref = atan2((y_goal - robot.y), (x_goal - robot.x));
    w = K1 * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
    phiR = (v + w * L) / r;
    phiL = (v - w * L) / r;

    motorR.setMotorSpeed(phiR);
    motorL.setMotorSpeed(phiL);
    x_int = robot.x + dist * 0.01 * cos(robot.theta) - 0.3 * sin(robot.theta);  //Utile pour l'évitement d'obstacle
    y_int = robot.y + dist * 0.01 * sin(robot.theta) + 0.3 * cos(robot.theta);  //Il sert à calculer un nouveau point à côté de l'obstacle
  }

  if (State == 5)  //Évitement d'obstacle (méthode de Kinjy, Lilian et Guillaume en robotique mais à modifier par la team 4 plus tard)
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

  if (State == 20)  //STOP
  {
    Serial.println("STOP");
    motorR.setMotorSpeed(0);
    motorL.setMotorSpeed(0);
  }

  //Transitions
  if (State == 0 && dist > 30) { State = 4; }
  if (State == 4 && (absf(robot.y - y_goal) < eps) && (absf(robot.x - x_goal)) < eps && i_goal != arrayLength) { State = 4; i_goal = i_goal + 1;}                                                                                                                             //Utiliser absf pour calculer une valeur absolue
  if (State == 4 && (absf(robot.y - y_goal) < eps) && (absf(robot.x - x_goal)) < eps && i_goal == arrayLength) { State = 20; }  //Jsp pourquoi la fct abs de Arduino ne marche pas correctement
  if (State == 4 && dist < 50) { State = 5; }
  if (State == 5 && (absf(robot.y - y_int) < eps) && (absf(robot.x - x_int)) < eps) { State = 4; }
  iter = iter + 1;
}

float absf(float val) {
  return val < 0 ? -val : val;
}

void  wait (int status) {  //Fonction pour la tirette
  while(digitalRead(pinTirette) != status){
    delay(100);
  }
}

void readEncoderD() {  //Tout est dans le nom
  motorR.readEncoder();
}
void readEncoderG() {
  motorL.readEncoder();
}