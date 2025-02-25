#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>
#include "Component.h"

class UltrasonicSensor : public Component {
private:
    int trigPin;
    int echoPin;
<<<<<<< Updated upstream
    double distance;

public:
    UltrasonicSensor(String name, int trig, int echo) : Component(name), trigPin(trig), echoPin(echo), distance(0) {
=======
    double distanceCm;
    unsigned long maxTimeoutMicroSec = 0;
    const double speedOfSoundInCmPerMicroSec = 0.034;
    const double maxDistanceCm = 400;

public:
    UltrasonicSensor(String name, int trig, int echo) : Component(name), trigPin(trig), echoPin(echo), maxDistanceCm(0) {
>>>>>>> Stashed changes
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    void update() override {
<<<<<<< Updated upstream
=======
        unsigned long maxDistanceDurationMicroSec;

>>>>>>> Stashed changes
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

<<<<<<< Updated upstream
        long duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.034 / 2;  // Convert to cm
    }

    double getDistance() { return distance; }
=======
        // Compute max delay based on max distance with 25% margin in microseconds
        maxDistanceDurationMicroSec = 2.5 * maxDistanceCm / speedOfSoundInCmPerMicroSec;
        if (maxTimeoutMicroSec > 0) {
            maxDistanceDurationMicroSec = min(maxDistanceDurationMicroSec, maxTimeoutMicroSec);
        }

        // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
        unsigned long durationMicroSec = pulseIn(echoPin, HIGH, maxDistanceDurationMicroSec); // can't measure beyond max distance

        float distanceCm = durationMicroSec / 2.0 * speedOfSoundInCmPerMicroSec;
    }

    double getDistance() { return distanceCm; }
>>>>>>> Stashed changes
};

#endif // ULTRASONICSENSOR_H
