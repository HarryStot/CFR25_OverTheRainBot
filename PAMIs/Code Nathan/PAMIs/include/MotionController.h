#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include <Arduino.h>
#include "Odometry.h"
#include "Trajectory.h"

class MotionController {
private:
    double speedGain;
    double rotationGain;

public:
    MotionController(double speedGain = 1.0, double rotationGain = 1.5)
        : speedGain(speedGain), rotationGain(rotationGain) {}

    void computeSpeed(double currentX, double currentY, double currentTheta,
                      Waypoint target, double &leftSpeed, double &rightSpeed) {

        double errorX = target.x - currentX;
        double errorY = target.y - currentY;
        double errorTheta = atan2(errorY, errorX) - currentTheta;
        
        // TODO : Change formula
        double forwardSpeed = sqrt(errorX * errorX + errorY * errorY) * speedGain;
        double rotationSpeed = errorTheta * rotationGain;

        leftSpeed = forwardSpeed - rotationSpeed;
        rightSpeed = forwardSpeed + rotationSpeed;
    }
};

#endif // MOTIONCONTROLLER_H
