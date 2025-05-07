#include "Odometry.h"

Odometry::Odometry(float L, float r) {
    this->L = L;
    this->r = r;
}

void Odometry::updateOdometry(float posR, float posL) {
    DR = r * (this.posR - this.posR_prec) * pi / 180;
    DL = r * (this.posL - this.posL_prec) * pi / 180;
    DC = (DR + DL) / 2;

    this.x = this.x_prec + DC * cos(theta);
    this.y = this.y_prec + DC * sin(theta);

    this.theta = this.theta_prec + (DR - DL) / (2 * L);
    this.theta_prec = this.theta;

    this.x_prec = this.x;
    this.y_prec = this.y;

    this.posL_prec = this.posL;
    this.posR_prec = this.posR;
}

void Odometry::setPosition(float new_x, float new_y, float new_theta) {
    x = new_x;
    y = new_y;
    theta = new_theta;
    
    x_prec = new_x;
    y_prec = new_y;
    theta_prec = new_theta;
    
    // Note: We don't reset encoder positions since they are hardware counters
    // The next updateOdometry call will use the current encoder values as reference
}
