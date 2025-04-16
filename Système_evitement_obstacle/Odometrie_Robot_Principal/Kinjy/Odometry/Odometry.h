#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

class Odometry {
public:
    float pi = 3.14, L, r;
    float posL = 0, posR = 0, theta_prec = pi / 2, DR = 0, DL = 0, DC = 0, x_prec = 0, y_prec = 0, posL_prec = 0, posR_prec = 0;
    float x = 0, y = 0, theta = 0;

    Odometry() {}
    Odometry(float L, float r);
    void updateOdometry(float posR, float posL);
};

#endif
