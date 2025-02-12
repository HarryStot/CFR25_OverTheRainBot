#include "src/Motor.h"

#define ENCODER_L_A 2
#define ENCODER_L_B 4

Motor motor("motor", 2, 3, 4);
Encoder encoder("encoder");

bool enabled;
bool direction;

void setup() {
    Serial.begin(115200);
    motor.setDirection(true, true);
    motor.setSpeed(255);
}

void loop() {
    motor.update();
    delay(1000);
    motor.getDirection(enabled, direction);
    Serial.println("Motor direction: " + String(direction));
    Serial.println("Motor speed: " + String(motor.getSpeed()));
}