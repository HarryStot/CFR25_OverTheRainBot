#include "Odometry.h"

Odometry::Odometry(float L, float r) {
    this->L = L;
    this->r = r;
}

void Odometry::updateOdometry(float posR, float posL) {
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
}

void Odometry::setPosition(float new_x, float new_y, float new_theta) {
    // Update all position variables
    x = new_x;
    y = new_y;
    theta = new_theta;
    
    // Update previous values to avoid jumps in the next odometry calculation
    x_prec = new_x;
    y_prec = new_y;
    theta_prec = new_theta;
    
    // Note: We don't reset encoder positions since they are hardware counters
    // The next updateOdometry call will use the current encoder values as reference
}
