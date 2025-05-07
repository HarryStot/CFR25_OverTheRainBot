#ifndef ENCODER_H
#define ENCODER_H

#include "Component.h"

class Encoder : public Component {
private:
    volatile int count;

public:
    Encoder(String name) : Component(name), count(0) {}

    void update_count (bool a, bool b) {
		static unsigned long lastInterruptTime = 0;
		unsigned long interruptTime = millis();
		
		if (interruptTime - lastInterruptTime > 5) {
			if (a == b) {
				count++;  // Clockwise
			} else {
				count--;  // Counterclockwise
			}
			lastInterruptTime = interruptTime;
		}
		
    }

    int getTicks() const { return count; }

    void reset() { count = 0; }

    void update() override {}
    
};

#endif // ENCODER_H
