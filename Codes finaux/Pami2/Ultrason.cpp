#include "Ultrason.h"

Ultrason::Ultrason(int trig, int echo) {
    trigPin = trig;
    echoPin = echo;
}

void Ultrason::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float Ultrason::readDistanceCM() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    if (duration == 0) return -1; // Rien détecté ou timeout
    return duration * 0.034 / 2.0;
}
