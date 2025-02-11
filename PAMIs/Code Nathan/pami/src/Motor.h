#ifndef MOTOR_H
#define MOTOR_H

#include "Component.h"
#include <Arduino.h>

class Motor : public Component {
private:
    int speed;

public:
    Motor(String name) : Component(name), speed(0) {}

    void setSpeed(int speed) { this->speed = speed; }
    int getSpeed() const { return speed; }

    void update() override {
        // Example: Print motor speed
        Serial.print(name + " Motor speed: ");
        Serial.println(speed);
    }
};

#endif // MOTOR_H
