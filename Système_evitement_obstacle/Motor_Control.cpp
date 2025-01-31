#include "Motor_Control.h"

Motor::Motor(byte enca, byte encb, byte pwm, byte dir) {
  this->enca = enca;
  this->encb = encb;
  this->u = u;
  this->pwm = pwm;
  this->dir = dir;
}

void Motor::init() {
  pinMode(enca, INPUT_PULLUP);
  pinMode(encb, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}

void Motor::setMotorSpeed(int phi) {
  int pwr = 0.7 * fabs(phi); //Equation à modifier selon l'étalonnage permettant de trouver la zone de fonctionnement linéaire

  if (pwr > 255) {
    pwr = 255;
  } else if (pwr < 50) { //Valeur de tension en dessous de laquelle le moteur ne tourne plus, obtenue grossièrement, à vérifier
    pwr = 0;
  }

  int sign = 1;
  if (phi < 0) {
    sign = -1;
  }

  analogWrite(pwm, pwr);

  if (sign == 1) {
    digitalWrite(dir, HIGH);
  } else {
    digitalWrite(dir, LOW);
  }
}

void Motor::readEncoder() {
  int d = digitalRead(encb);
  if (d > 0) {
    pos++;
  } else {
    pos--;
  }
}