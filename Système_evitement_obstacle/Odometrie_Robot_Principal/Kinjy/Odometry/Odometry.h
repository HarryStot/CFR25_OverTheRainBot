#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

class Odometry {
public:
    float pi = 3.14, L, r;
    float posL = 0, posR = 0, theta_prec = 0, theta = 0, x = 0, y = 0, DR = 0, DL = 0, DC = 0, x_prec = 0, y_prec = 0, posL_prec = 0, posR_prec = 0;

    Odometry() {}
    Odometry(float L, float r);
    void init(float x_init, float y_init, float theta_init);
    void updateOdometry(float posR, float posL);
    void setPosition(float new_x, float new_y, float new_theta);
};

#endif