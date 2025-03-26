#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

class Odometry {
public:
    float x = 0, y = 0, theta = 0;
    Odometry(float L, float r);
    void updateOdometry(float posR, float posL);
};

#endif
