#include "Odometry.h"

Odometry::Odometry(float L, float r) {
    this->L = L;
    this->r = r;
}

void Odometry::updateOdometry(float posR, float posL) {
    float DR = r * (posR - posR_prec) * 3.14 / 180;
    float DL = r * (posL - posL_prec) * 3.14 / 180;
    float DC = (DR + DL) / 2;

    x += DC * cos(theta);
    y += DC * sin(theta);
    theta += (DR - DL) / (2 * L);

    posR_prec = posR;
    posL_prec = posL;
}
