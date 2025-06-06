#include "Odometry.h"

void Odometry::init(float x_init, float y_init, float theta_init) {
    x_prec = x_init;
    y_prec = y_init;
    theta_prec = theta_init;
}

Odometry::Odometry(float L, float r) {
    this->L = L;
    this->r = r;
}

void Odometry::updateOdometry(float posR, float posL) {
    DR = r * (posR - posR_prec) * 2 * pi / 699; //2 * pi * r * delta_tick / N
    DL = r * (posL - posL_prec) * 2 * pi / 699;
    DC = (DR + DL) / 2;

    x = x_prec + DC * cos(theta);
    y = y_prec + DC * sin(theta);

    theta = theta_prec + (DR - DL) / (2 * L);
    theta_prec = theta;

    x_prec = x;
    y_prec = y;

    posL_prec = posL;
    posR_prec = posR;
}
