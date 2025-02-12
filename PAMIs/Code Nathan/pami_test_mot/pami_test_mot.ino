#include "src/Wheel.h"

#define ENCODER_L_A 2
#define ENCODER_L_B 3

Wheel* wheel;

unsigned long last_time = 0;

void leftEncoderISR() {
    wheel->updateEncoder(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
}

void setup() {
    Serial.begin(115200);

    wheel = new Wheel("wheel", 4, 5, 6);

    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);
}

void loop() {
    wheel->update();
    last_time = millis();
    
    Serial.println(wheel->getEncoderValue());
}