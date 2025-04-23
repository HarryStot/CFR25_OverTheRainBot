#include "src/Robot.h"

#define ENCODER_L_B 11
#define ENCODER_L_A 2
#define ENCODER_R_B 12
#define ENCODER_R_A 3

const double WHEEL_RADIUS = 0.04;
const double WHEEL_BASE = 0.10;

Robot* robot = nullptr;

//Robot robot(WHEEL_RADIUS, WHEEL_BASE);

Wheel* wheelL;
Wheel* wheelR;

void leftEncoderISR() {
    wheelL->updateEncoder(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
	//Serial.print("Left Encoder Count: ");
    //Serial.println(wheelL->getEncoderValue());
}

void rightEncoderISR() {
    wheelR->updateEncoder(digitalRead(ENCODER_R_A), digitalRead(ENCODER_R_B));
	//Serial.print("Right Encoder Count: ");
    //Serial.println(wheelR->getEncoderValue());
}

void setup() {
	Serial.begin(115200);

	robot = new Robot(WHEEL_RADIUS, WHEEL_BASE);
	
	Serial.println("Robot initialized.");


    wheelL = new Wheel("wheelL", 4, 5, 10);
	wheelR = new Wheel("wheelR", 6, 7, 9);
	
	robot->addComponent("wheelL", wheelL);
    robot->addComponent("wheelR", wheelR);

    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);
	pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, RISING);
	
	robot->addWaypoint(0.3, 0, 0);
}

void loop() {
	robot->updateAll();
	
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's') {
            robot->stop();
        }
    }
	
	// TODO: Add manual control
    delay(10);
}