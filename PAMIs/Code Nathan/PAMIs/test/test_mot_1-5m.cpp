#include <Arduino.h>
#include "Robot.h"
#include "Tirette.h"

#define ENCODER_L_A 2
#define ENCODER_L_B 4
#define ENCODER_R_A 3
#define ENCODER_R_B 5
#define ULTRASONIC_TRIG 6
#define ULTRASONIC_ECHO 7
#define TIRETTE_PIN 13

const double WHEEL_RADIUS = 0.03;
const double WHEEL_BASE = 0.15;

Robot robot(WHEEL_RADIUS, WHEEL_BASE);
Wheel* wheelL;
Wheel* wheelR;
Tirette tirette(TIRETTE_PIN);

void leftEncoderISR() {
    wheelL->updateEncoder(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
}

void rightEncoderISR() {
    wheelR->updateEncoder(digitalRead(ENCODER_R_A), digitalRead(ENCODER_R_B));
}

void setup() {
    Serial.begin(115200);

    wheelL = new Wheel("Left_Wheel", 4, 5, 6); // TODO : change pins
    wheelR = new Wheel("Right_Wheel", 4, 5, 6);

    robot.addComponent("wheelL", wheelL);
    robot.addComponent("wheelR", wheelR);

    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);
    pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, CHANGE);

    robot.addWaypoint(1.5, 0, 0);

    Serial.println("Ready");
    
    tirette.wait(0);
    Serial.println("Initialisation");

    tirette.wait(1);
    Serial.println("Go");

    robot.setStartTime(millis());
}

void loop() {
    if (millis() - robot.getStartTime() >= 85000) {
        robot.updateAll();   
    }

    if (millis() - robot.getStartTime() >= robot.getMaxTime() - 1000) {
        robot.stop();
    }

    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's') {
            robot.stop();
        }
    }
    delay(10);
}