#include "Motor_Control.h"

Motor::Motor(byte enca, byte encb, byte pwm, byte dir) {
    this->enca = enca;
    this->encb = encb;
    this->pwm = pwm;
    this->dir = dir;
}

void Motor::init() {
    pinMode(enca, INPUT_PULLUP);
    pinMode(encb, INPUT_PULLUP);
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
}

void Motor::setMotorSpeed(int phi) {
    int pwr = min(255, max(0, int(0.7 * fabs(phi))));
    analogWrite(pwm, pwr);
    digitalWrite(dir, phi >= 0 ? HIGH : LOW);
}

void Motor::readEncoder() {
    pos += digitalRead(encb) ? 1 : -1;
}
