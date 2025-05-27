#include "Motor_Control.h"

Motor::Motor(byte enca, byte encb, byte pwm, byte dir, byte brkd, byte brkg) {
    this->enca = enca;
    this->encb = encb;
    this->pwm = pwm;
    this->dir = dir;
    this->brkd = brkd;
    this->brkg = brkg;

    this->lastUpdateTime = 0;
}

void Motor::init() {
    pinMode(enca, INPUT_PULLUP);
    pinMode(encb, INPUT_PULLUP);
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(brkd, OUTPUT);
    pinMode(brkg, OUTPUT);
    
    // Désactivation des freins au démarrage
    digitalWrite(brkd, LOW);
    digitalWrite(brkg, LOW);
}

void Motor::setMotorSpeed(float phi) {
    int targetPwr = (int)phi;
    if (targetPwr < 0) targetPwr = -targetPwr;
    if (targetPwr > 255) targetPwr = 255;

    bool direction = phi >= 0;
    // Serial.println(targetPwr);
    // Définir la direction
    digitalWrite(dir, direction ? HIGH : LOW);

    digitalWrite(brkd, LOW); // Désactivation du frein
    digitalWrite(brkg, LOW); 
    
    analogWrite(pwm, targetPwr);
}

void Motor::readEncoder() {
    pos += digitalRead(encb) ? 1 : -1;
}
