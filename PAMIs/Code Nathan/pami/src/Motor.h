#ifndef MOTOR_H
#define MOTOR_H

#include "Component.h"
#include <Arduino.h>

class Motor : public Component {
private:
    int speed;
    bool direction;
    bool enabled;
    int pinA;
    int pinB;
    int pinPWM;

public:
    Motor(String name, int pinA, int pinB, int pinPWM) : Component(name), speed(0), direction(true), enabled(false) {
        pinMode(pinA, OUTPUT);
        pinMode(pinB, OUTPUT);
        pinMode(pinPWM, OUTPUT);
        this->pinA = pinA;
        this->pinB = pinB;
        this->pinPWM = pinPWM;
    }

    void setSpeed(int speed) { this->speed = speed; }

    int getSpeed() const { return speed; }

    void setDirection(bool enabled, bool direction) { 
        this->direction = direction;
        this->enabled = enabled;
        if (enabled) {
            digitalWrite(pinA, direction);
            digitalWrite(pinB, !direction);
        } else {
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, LOW);
        }
    }

    void update() override {
        // TODO Implement motor control with PWM
        analogWrite(pinPWM, speed);
    }
};

#endif // MOTOR_H
