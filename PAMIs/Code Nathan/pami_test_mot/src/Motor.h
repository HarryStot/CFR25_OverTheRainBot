#ifndef MOTOR_H
#define MOTOR_H

#include "Component.h"
#include <Arduino.h>

class Motor : public Component {
private:
    float speed; // Speed in m/s ?
    bool direction;
    bool enabled;
    int pinA;
    int pinB;
    int pinPWM;
	const float A;

public:
    Motor(String name, int pinA, int pinB, int pinPWM, float A) : Component(name), speed(0), direction(true), enabled(false), A(A) {
        pinMode(pinA, OUTPUT);
        pinMode(pinB, OUTPUT);
        pinMode(pinPWM, OUTPUT);
        this->pinA = pinA;
        this->pinB = pinB;
        this->pinPWM = pinPWM;
    }

    void setSpeed(float speed) { this->speed = speed;} 

    float getSpeed() const { return speed; }

    void setDirection(bool enabled, bool direction) { 
        this->direction = direction;
        this->enabled = enabled;
        if (enabled) {
            digitalWrite(pinA, !direction); // for L298n clasic
            digitalWrite(pinB, direction);
            
        } else {
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, LOW);
        }
    }

    void getDirection(bool &enabled, bool &direction) const {
        enabled = this->enabled;
        direction = this->direction;
    }

    void update() override {
        if (enabled) {
            if (speed > 0) {
                this->direction = true;
                this->setDirection(enabled, direction);
            } else if (speed < 0) {
                this->direction = false;
                this->setDirection(enabled, direction);
            }
            
			int pwmValue = (int) constrain(A * speed, 0, 255);

            // Dynamic gain
            // if (pwmValue < 50) {
            //     pwmValue = (int) constrain(2 * A * speed, 0, 255);
            // } else if (pwmValue > 200) {
            //     pwmValue = (int) constrain(0.75 * A * speed, 0, 255);
            // }


            analogWrite(pinPWM, pwmValue);
            
			// Serial.print("PWM: ");
			// Serial.println(pwmValue);
			
        } else {
            analogWrite(pinPWM, 0); 
        }
    }
	
};

#endif // MOTOR_H
