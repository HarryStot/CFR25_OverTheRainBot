#ifndef MOTORUP_H
#define MOTORUP_H

#include "Component.h"
#include <Arduino.h>

class MotorUP : public Component {
private:
    int speed;
    int pin;
    bool enabled;
        
public:
    MotorUP(String name, int pin) : Component(name), speed(0), pin(pin), enabled(false) {
        pinMode(pin, OUTPUT);
        this->pin = pin;
    }

    void setSpeed(int speed) { this->speed = speed;} 

    int getSpeed() const { return speed; }

    void update() override {
        if (enabled) {
			int Value = constrain(speed, 0, 255);
            analogWrite(pin, Value);  
            
			Serial.print(" Value ");
			Serial.println(Value);
			
        } else {
            analogWrite(pin, 0); 
        }
    }
	
};

#endif // MOTORUP_H


