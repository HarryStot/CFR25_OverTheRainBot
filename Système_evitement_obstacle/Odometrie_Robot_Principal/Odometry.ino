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

Motor motorR(ENCAD, ENCBD, PWMD, DIRD);
Motor motorL(ENCAG, ENCBG, PWMG, DIRG);

float phiR = 0.0, phiL = 0.0;
float L = 0.245 / 2;
float r = 0.065 / 2;
float v, w;
float posL = 0, posR = 0;
float thetaref;
float eps = 0.1;
float x_int, y_int;
float pi = 3.14;
float chrono;

int iter = 0; 
const uint8_t pinTirette = 27;
double dist;
float K1 = 300;

float ly_goal[4] = { 0, 1, 1, 0 }; //Prévoir le point de retour à la base 0,0
float lx_goal[4] = { 0, 0, 2, 0 };
int arrayLength = sizeof(lx_goal) / sizeof(lx_goal[0]);
float x_goal, y_goal;
int i_goal = 0;

Odometry robot(L, r);

enum StateType { DEPART, NAVIGATION, EVITEMENT, STOP };
StateType State = DEPART;

void setup() {
  Serial.begin(9600);
  pinMode(pinTirette, INPUT_PULLUP);

  wait(0);
  Serial.println("Initialisation");
  wait(1);
  Serial.println("Match");

  motorR.init();
  attachInterrupt(digitalPinToInterrupt(ENCAD), readEncoderD, RISING);
  motorL.init();
  attachInterrupt(digitalPinToInterrupt(ENCAG), readEncoderG, RISING);
  
  delay(1500);
  chrono = millis();
}

void loop() {
  Serial.println(State);
  Serial.println(dist);
  
  dist = 999;
  posR = -motorR.pos;
  posL = motorL.pos;
  robot.updateOdometry(posR, posL, x_goal, y_goal);

  switch (State) {
    case DEPART:
      motorR.setMotorSpeed(0);
      motorL.setMotorSpeed(0);
      break;
    
    case NAVIGATION:
      x_goal = lx_goal[i_goal];
      y_goal = ly_goal[i_goal];
      thetaref = atan2((y_goal - robot.y), (x_goal - robot.x));
      w = K1 * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
      if (absf(thetaref - robot.theta) > eps) {
        v = 0;
        phiR = (v + w * L) / r;
        phiL = (v - w * L) / r;
        motorR.setMotorSpeed(phiR);
        motorL.setMotorSpeed(phiL);
      } else {
        v = 10;
        phiR = (v + w * L) / r;
        phiL = (v - w * L) / r;
        motorR.setMotorSpeed(phiR);
        motorL.setMotorSpeed(phiL);
      }
      break;
    
    case EVITEMENT:
      v = 0;
      w = 0;
      phiR = (v + w * L) / r;
      phiL = (v - w * L) / r;
      motorR.setMotorSpeed(phiR);
      motorL.setMotorSpeed(phiL);
      break;
    
    case STOP:
      Serial.println("STOP");
      motorR.setMotorSpeed(0);
      motorL.setMotorSpeed(0);
      break;
  }

  if (State == DEPART && dist > 30) { State = NAVIGATION; }
  if (State == NAVIGATION && (absf(robot.y - y_goal) < eps) && (absf(robot.x - x_goal)) < eps && i_goal != arrayLength) { State = NAVIGATION; i_goal++; }
  if (State == NAVIGATION && (absf(robot.y - y_goal) < eps) && (absf(robot.x - x_goal)) < eps && i_goal == arrayLength) { State = STOP; }
  if (State == NAVIGATION && dist > 0 && dist < 50) { State = EVITEMENT; }
  if (State == EVITEMENT && dist > 50) { State = NAVIGATION; }
  if (millis() - chrono > 85000) { State = NAVIGATION; i_goal = arrayLength;}
  if (millis() - chrono > 99000) { State = STOP; }
  
  iter++;
}

float absf(float val) {
  return val < 0 ? -val : val;
}

void wait(int status) {
  while (digitalRead(pinTirette) != status) {
    delay(100);
  }
}

void readEncoderD() {
  motorR.readEncoder();
}
void readEncoderG() {
  motorL.readEncoder();
}
