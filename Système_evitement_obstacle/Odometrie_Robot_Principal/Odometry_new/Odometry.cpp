#include "Odometry.h"

Odometry::Odometry(float L, float r) {
  this->L = L;
  this->r = r;
}

void Odometry::updateOdometry(float posR, float posL, float x_goal, float y_goal) { //Voir le cours de robotique

  DR = r * (posR - posR_prec) * pi / 180;
  DL = r * (posL - posL_prec) * pi / 180;
  DC = (DR + DL) / 2;

  x = x_prec + DC * cos(theta);
  y = y_prec + DC * sin(theta);

  theta = theta_prec + (DR - DL) / (2 * L);
  theta_prec = theta;

  x_prec = x;
  y_prec = y;

  posL_prec = posL;
  posR_prec = posR;

  Serial.print("x_goal = ");
  Serial.print(x_goal);
  Serial.print("   ");
  Serial.print("y_goal = ");
  Serial.print(y_goal);
  Serial.print("   ");
  Serial.print("x = ");
  Serial.print(x);
  Serial.print("   ");
  Serial.print("y = ");
  Serial.println(y);
}