#include "src/Wheel.h"

#define ENCODER_L_B 3
#define ENCODER_L_A 2
#define ENCODER_R_B 11
#define ENCODER_R_A 12

Wheel* wheel;

unsigned long last_time = 0;

void leftEncoderISR() {
    wheel_L->updateEncoder(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
}

void rightEncoderISR() {
    wheel_L->updateEncoder(digitalRead(ENCODER_R_A), digitalRead(ENCODER_R_B));
}

void setup() {
    Serial.begin(115200);

    wheel = new Wheel_L("wheel_L", 4, 5, 10);
	wheel = new Wheel_R("wheel_R", 5, 6, 9);

    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);
	pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, RISING);
}

void loop() {
    wheel->update();
    last_time = millis();
    
    Serial.println(wheel_L->getEncoderValue());
	Serial.println(wheel_R->getEncoderValue());
}