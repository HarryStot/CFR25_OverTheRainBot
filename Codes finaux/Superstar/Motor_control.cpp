#include "Motor_Control.h"

Motor::Motor(byte enca, byte encb, byte pwm, byte pina, byte pinb) {
    this->enca = enca;
    this->encb = encb;
    this->pwm = pwm;
    this->pina = pina;
    this->pinb = pinb;

    this->lastUpdateTime = 0;
}

void Motor::init() {
    pinMode(enca, INPUT_PULLUP);
    pinMode(encb, INPUT_PULLUP);
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(pina, OUTPUT);
    pinMode(pinb, OUTPUT);
}

void Motor::setMotorSpeed(float phi) {
    int targetPwr = (int)phi;
    if (targetPwr < 0) targetPwr = -targetPwr;
    if (targetPwr > 180) targetPwr = 180;

    bool direction = phi >= 0;
    // Serial.println(targetPwr);
    digitalWrite(pina, !direction);
    digitalWrite(pinb, direction);
    analogWrite(pwm, targetPwr);
}

void Motor::readEncoder() {
    pos += digitalRead(encb) ? 1 : -1;
}
