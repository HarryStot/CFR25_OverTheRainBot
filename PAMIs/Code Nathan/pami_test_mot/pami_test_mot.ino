#include "src/Wheel.h"

#define ENCODER_L_A 2
#define ENCODER_L_B 11
#define ENCODER_R_A 3
#define ENCODER_R_B 12

Wheel* wheel;

Encoder encoder_L("encoder_L");
Encoder encoder_R("encoder_R");

unsigned long last_time = 0;

void leftEncoderISR() {
    wheel->updateEncoder(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
}

void rightEncoderISR() {
    encoder_R.update_count(digitalRead(ENCODER_R_A), digitalRead(ENCODER_R_B));
}
void setup() {
    Serial.begin(115200);

    wheel = new Wheel("wheel", 4, 5, 6);

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
    
    Serial.println(wheel->getEncoderValue());
}
