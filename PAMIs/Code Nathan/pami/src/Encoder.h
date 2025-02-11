#ifndef ENCODER_H
#define ENCODER_H

#include "Component.h"

class Encoder : public Component {
private:
    volatile int count;

public:
    Encoder(String name) : Component(name), count(0) {}

    void update(bool a, bool b) {
        if (a == b) count++;  // Clockwise
        else count--;         // Counterclockwise
    }

    int getTicks() const { return count; }

    void reset() { count = 0; }

    void update() override {
        Serial.print(name + " Encoder Count: ");
        Serial.println(count);
    }
};

#endif // ENCODER_H
