#ifndef WHEEL_H
#define WHEEL_H

#include "Motor.h"
#include "Encoder.h"

class Wheel : public Component {
private:
    Motor motor;
    Encoder encoder;

public:
    Wheel(String name) : Component(name), motor(name + "_Motor"), encoder(name + "_Encoder") {}

    // TODO Change code
    void setSpeed(int speed) { motor.setSpeed(speed); }
    int getSpeed() const { return motor.getSpeed(); }

    void updateEncoder(bool a, bool b) { encoder.update(a, b); }
    int getEncoderValue() const { return encoder.getTicks(); }

    void resetEncoder() { encoder.reset(); }

    void update() override {
        motor.update();
        encoder.update();
    }
};

#endif // WHEEL_H
