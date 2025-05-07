#ifndef TIRETTE_H
#define TIRETTE_H

#include <Arduino.h>

class Tirette {
private:
    int pin;
    bool isPulled = false;

public:
    Tirette(int pin) : pin(pin) {
        pinMode(pin, INPUT_PULLUP);
    }

    bool isPulled() {
        return digitalRead(pin) == LOW;
    }

    void wait(int status) {
        while(digitalRead(pin) != status){
          delay(100);
        }
    }
};

#endif // TIRETTE_H