#ifndef ULTRASON_H
#define ULTRASON_H

#include <Arduino.h>

class Ultrason {
private:
    int trigPin;
    int echoPin;

public:
    Ultrason(int trig, int echo);
    void begin();
    float readDistanceCM();
};

#endif