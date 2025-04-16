#include "src/Robot.h"

#define ENCODER_L_B 11
#define ENCODER_L_A 2
#define ENCODER_R_B 12
#define ENCODER_R_A 3

const double WHEEL_RADIUS = 0.04;
const double WHEEL_BASE = 0.15;

Robot robot(WHEEL_RADIUS, WHEEL_BASE);

Wheel* wheel_L;
Wheel* wheel_R;

void leftEncoderISR() {
    wheel_L->updateEncoder(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
}

void rightEncoderISR() {
    wheel_R->updateEncoder(digitalRead(ENCODER_R_A), digitalRead(ENCODER_R_B));
}

void setup() {
    Serial.begin(115200);

    wheel_L = new Wheel("wheel_L", 4, 5, 10);
	wheel_R = new Wheel("wheel_R", 6, 7, 9);
	
	robot.addComponent("wheelL", wheelL);
    robot.addComponent("wheelR", wheelR);

    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);
	pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, RISING);
	
	robot.addWaypoint(0.2, 0.2, 0);
}

void loop() {
	robot.updateAll();
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's') {
            robot.stop();
        }
    }
    // TODO: Add manual control
    delay(10);
	
    wheel_L->update();
	wheel_L->setSpeed(100);
	wheel_L->setDirection(true, false);
	
	wheel_R->update();
	wheel_R->setSpeed(100);
	wheel_R->setDirection(true, true);
	
    last_time = millis();
    
    Serial.println(wheel_L->getEncoderValue());
	Serial.println(wheel_R->getEncoderValue());
}