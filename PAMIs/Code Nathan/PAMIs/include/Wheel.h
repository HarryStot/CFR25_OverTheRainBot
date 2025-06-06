#ifndef WHEEL_H
#define WHEEL_H

#include "Motor.h"
#include "Encoder.h"

class Wheel : public Component {
private:
    Motor motor;
    Encoder encoder;

public:
    Wheel(String name, int pinA, int pinB, int pinPWM) : Component(name), motor(name + "_Motor", pinA, pinB, pinPWM), encoder(name + "_Encoder") {
        motor.setSpeed(0);
        motor.setDirection(false, false);
    }

    // TODO Change code
    void setSpeed(int speed) { motor.setSpeed(speed); }
    int getSpeed() const { return motor.getSpeed(); }

    void updateEncoder(bool a, bool b) { encoder.update_count(a, b); }
    int getEncoderValue() const { return encoder.getTicks(); }

    void resetEncoder() { encoder.reset(); }

    void update() override {
        motor.update();
        encoder.update();
    }
};

#endif // WHEEL_H
