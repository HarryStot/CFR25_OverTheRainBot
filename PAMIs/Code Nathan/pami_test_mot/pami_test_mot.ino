#include "src/Motor.h"
#include "src/Encoder.h"

#define ENCODER_L_A 2
#define ENCODER_L_B 3

Motor motor("motor", 4, 5, 6);
Encoder encoder("encoder");

unsigned long last_time = 0;


void leftEncoderISR() {
    encoder.update_count(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
}

void setup() {
    Serial.begin(115200);
    motor.setDirection(true, false);
    motor.setSpeed(255);

    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);
}

void loop() {
    motor.update();
    last_time = millis();
    
    Serial.println(encoder.getTicks());
}