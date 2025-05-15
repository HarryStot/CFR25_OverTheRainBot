#ifndef WHEEL_H
#define WHEEL_H

#include "Motor.h"
#include "Encoder.h"

class Wheel : public Component {
private:
    Motor* motor;
    Encoder encoder;

public:
	// Better construct
    Wheel(String name, int pinA, int pinB, int pinPWM) : Component(name), motor(new Motor(name + "_Motor", pinA, pinB, pinPWM, 55)), encoder(name + "_Encoder") {
        motor->setSpeed(0);
        motor->setDirection(false, false);
    }

    void setSpeed(float speed) { motor->setSpeed(speed); }
    int getSpeed() const { return motor->getSpeed(); }
	
	void setDirection(bool enabled, bool direction) { motor->setDirection(enabled, direction); }
	void getDirection(bool &enabled, bool &direction) const { motor->getDirection(enabled, direction); }

    void updateEncoder(bool a, bool b) { encoder.update_count(a, b); }
    int getEncoderValue() const { return encoder.getTicks(); }

    void resetEncoder() { encoder.reset(); }

    Encoder getEncoder() const { return encoder; }

    void update() override {
        motor->update();
        encoder.update();
    }
};

#endif // WHEEL_H
