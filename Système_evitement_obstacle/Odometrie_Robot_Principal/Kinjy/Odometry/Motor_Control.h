#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class Motor {
private:
    byte enca, encb, pwm, dir, brkd, brkg;
    unsigned long lastUpdateTime = 0;
public:
    volatile int pos;

    Motor(byte enca, byte encb, byte pwm, byte dir, byte brkd, byte brkg);
    void init();
    void setMotorSpeed(float phi);
    void readEncoder();
};

#endif
