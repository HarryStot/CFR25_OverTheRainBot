#include <Arduino.h>
#include "src/Robot.h"
#include <HardwareSerial.h>

#define ENCODER_L_A 2
#define ENCODER_L_B 11
#define ENCODER_R_A 3
#define ENCODER_R_B 12
#define ULTRASONIC_TRIG A0
#define ULTRASONIC_ECHO A1

const double WHEEL_RADIUS = 0.04;
const double WHEEL_BASE = 0.10;

Robot* robot;

// Robot robot(WHEEL_RADIUS, WHEEL_BASE);

Wheel* wheelL;
Wheel* wheelR;
UltrasonicSensor* ultrasonic;
MotorUP* motorUP;

void leftEncoderISR() {
    wheelL->updateEncoder(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
	// Serial.print("Left Encoder Count: ");
    // Serial.println(wheelL->getEncoderValue());
}

void rightEncoderISR() {
    wheelR->updateEncoder(digitalRead(ENCODER_R_A), digitalRead(ENCODER_R_B));
    // Serial.print("Right Encoder Count: ");
    // Serial.println(wheelR->getEncoderValue());
}


void setup() {
	Serial.begin(115200);
    
	robot = new Robot(WHEEL_RADIUS, WHEEL_BASE);
	
	Serial.println("Robot initialized.");

    wheelL = new Wheel("wheelL", 4, 5, 10);
	wheelR = new Wheel("wheelR", 6, 7, 9);

    ultrasonic = new UltrasonicSensor("Ultrasonic sensor", ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    motorUP = new MotorUP("MotorUP", 8);
	
	robot->addComponent("wheelL", wheelL);
    robot->addComponent("wheelR", wheelR);
	robot->addComponent("ultrasonic", ultrasonic);
    robot->addComponent("motorUP", motorUP);
	
    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);
    pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, RISING);
	
	robot->addWaypoint(1, 0, 0);

    // wheelL->setSpeed(150*2);
    // wheelR->setSpeed(150*2);
 
    wheelL->setDirection(true, true); // Forward
    wheelR->setDirection(true, true); // Forward
}
void loop() {
    // analogWrite(8, 100); 
    // delay(1000); // Wait for 1 second

    // analogWrite(8, 0); // Stop the motor    
    // delay(1000); // Wait for 1 second

    // Serial.println("Looping...");
	robot->updateAll();

    // robot->updateOdometry(wheelL->getEncoderValue(), wheelR->getEncoderValue());


    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's') {
            robot->stop();
            Serial.println("Robot stopped.");
        }
    }

	// TODO: Add manual control
    delay(10);
}
