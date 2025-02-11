#ifndef SENSOR_H
#define SENSOR_H

#include "Component.h"

class Sensor : public Component {
private:
    int value;
    int pin;

public:
    Sensor(String name, int pin) : Component(name), value(0), pin(pin) {}

    int readValue() {
        value = analogRead(pin);  // Example sensor reading
        return value;
    }

    void update() override {
        Serial.print(name + " Sensor Value: ");
        Serial.println(value);
    }
};

#endif // SENSOR_H
