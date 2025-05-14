#ifndef MOTORUP_H
#define MOTORUP_H

#include "Component.h"
#include <Arduino.h>

class MotorUP : public Component {
private:
    int pin;
    bool enabled;
        
public:
    MotorUP(String name, int pin) : Component(name), pin(pin), enabled(false) {
        pinMode(pin, OUTPUT);
        this->pin = pin;
    }

    void update() override {
        if (enabled) {
            digitalWriteWrite(pin, HIGH); // Set the pin HIGH to enable the motor
			
        } else {
            digitalWriteWrite(pin, LOW); // Set the pin LOW to disable the motor
        }
    }
	
};

#endif // MOTORUP_H


