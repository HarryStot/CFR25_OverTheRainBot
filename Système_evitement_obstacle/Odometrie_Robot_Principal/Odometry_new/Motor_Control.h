#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class Motor {
private:
    byte enca, encb, pwm, dir;
    unsigned long lastUpdateTime = 0;
    int currentPwr = 0;
public:
    volatile int pos;

    Motor(byte enca, byte encb, byte pwm, byte dir);
    void init();
    void setMotorSpeed(float phi);
    void readEncoder();
};

#endif
