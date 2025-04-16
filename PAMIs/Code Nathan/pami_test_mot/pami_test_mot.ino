#include "src/Motor.h"
#include "src/Encoder.h"

#define ENCODER_L_A 2
#define ENCODER_L_B 11
#define ENCODER_R_A 3
#define ENCODER_R_B 12

Motor motor_L("motor_L", 4, 5, 10);
Motor motor_R("motor_R", 6, 7, 9);

Encoder encoder_L("encoder_L");
Encoder encoder_R("encoder_R");

unsigned long last_time = 0;


void leftEncoderISR() {
    encoder_L.update_count(digitalRead(ENCODER_L_A), digitalRead(ENCODER_L_B));
}

void rightEncoderISR() {
    encoder_R.update_count(digitalRead(ENCODER_R_A), digitalRead(ENCODER_R_B));
}
void setup() {
    Serial.begin(115200);
    motor_L.setDirection(true, false);
    motor_L.setSpeed(255);

    motor_R.setDirection(true, false);
    motor_R.setSpeed(255);

    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_L_B, INPUT);

    pinMode(ENCODER_R_A, INPUT);
    pinMode(ENCODER_R_B, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, RISING);
}

void loop() {
    motor_L.update();
    motor_R.update();
    last_time = millis();
    
    Serial.println(encoder_L.getTicks());
    Serial.println(encoder_R.getTicks());
}
