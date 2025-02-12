#include "src/Robot.h"

#define ENCODER_L_A 2
#define ENCODER_L_B 4
#define ENCODER_R_A 3
#define ENCODER_R_B 5
#define ULTRASONIC_TRIG 6
#define ULTRASONIC_ECHO 7

const double WHEEL_RADIUS = 0.03;
const double WHEEL_BASE = 0.15;

Robot robot(WHEEL_RADIUS, WHEEL_BASE);
Wheel* wheelL;
Wheel* wheelR;
UltrasonicSensor* ultrasonic;

void leftEncoderISR() {
    wheelL->updateEncoder(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
}

void rightEncoderISR() {
    wheelR->updateEncoder(digitalRead(ENCODER_R_A), digitalRead(ENCODER_R_B));
}

void setup() {
    Serial.begin(115200);

    wheelL = new Wheel("Left_Wheel");
    wheelR = new Wheel("Right_Wheel");
    ultrasonic = new UltrasonicSensor("Ultrasonic sensor", ULTRASONIC_TRIG, ULTRASONIC_ECHO);

    robot.addComponent("wheelL", wheelL);
    robot.addComponent("wheelR", wheelR);
    robot.addComponent("ultrasonic", ultrasonic);

    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);
    pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, CHANGE);

    robot.addWaypoint(0.2, 0.2, 0);
    robot.addWaypoint(0.4, 0.0, 0);
    robot.addWaypoint(0.0, 0.0, 0);
}

void loop() {
    robot.updateAll();
    delay(50);
    // TODO Implement a way to stop the robot
}