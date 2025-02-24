#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>
#include "Component.h"

class UltrasonicSensor : public Component {
private:
    int trigPin;
    int echoPin;
    double distance;

public:
    UltrasonicSensor(String name, int trig, int echo) : Component(name), trigPin(trig), echoPin(echo), distance(0) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    void update() override {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.034 / 2;  // Convert to cm
    }

    double getDistance() { return distance; }
};

#endif // ULTRASONICSENSOR_H
