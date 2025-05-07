#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>
#include "Component.h"

class UltrasonicSensor : public Component {
private:
    int trigPin;
    int echoPin;
    double distanceCm;
    unsigned long maxTimeoutMicroSec = 0;
    const double speedOfSoundInCmPerMicroSec = 0.034;
    const double maxDistanceCm = 400;

public:
    UltrasonicSensor(String name, int trig, int echo) : Component(name), trigPin(trig), echoPin(echo), distanceCm(0) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    void update() override {
        unsigned long maxDistanceDurationMicroSec;

        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // Compute max delay based on max distance with 25% margin in microseconds
        maxDistanceDurationMicroSec = 2.5 * maxDistanceCm / speedOfSoundInCmPerMicroSec;
        if (maxTimeoutMicroSec > 0) {
            maxDistanceDurationMicroSec = min(maxDistanceDurationMicroSec, maxTimeoutMicroSec);
        }

        // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
        unsigned long durationMicroSec = pulseIn(echoPin, HIGH, maxDistanceDurationMicroSec); // can't measure beyond max distance

        
		distanceCm = durationMicroSec / 2.0 * speedOfSoundInCmPerMicroSec;
		
		 if (durationMicroSec == 0) {
			distanceCm = maxDistanceCm;
		} else {
			distanceCm = (durationMicroSec / 2.0) * speedOfSoundInCmPerMicroSec;
		}
        /*
		// Debug ici, correctement placé DANS une fonction
		Serial.print(name);  // ← OK ici
		Serial.print(" - Distance: ");
		Serial.print(distanceCm);
		Serial.println(" cm");
        */
		}
	
		
    double getDistance() { return distanceCm; }
};

#endif // ULTRASONICSENSOR_H
