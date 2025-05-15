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
        this->enabled = false;
        digitalWrite(pin, LOW); 
    }

    void enable() {
        enabled = true;
        update(); 
    }

    void disable() {
        enabled = false;
        update(); 
    }
    
    void update() override {
        if (enabled) {
            digitalWrite(pin, HIGH); // Set the pin HIGH to enable the motor
			
        } else {
            digitalWrite(pin, LOW); // Set the pin LOW to disable the motor
        }
    }
	
};

#endif // MOTORUP_H


