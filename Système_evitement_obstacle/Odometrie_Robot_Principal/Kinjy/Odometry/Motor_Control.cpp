#include "Motor_Control.h"

Motor::Motor(byte enca, byte encb, byte pwm, byte dir) {
    // Constructor to initialize the motor control pins
    this->enca = enca;
    this->encb = encb;
    this->pwm = pwm;
    this->dir = dir;
}

void Motor::init() {
    // Initialize the motor control pins
    pinMode(enca, INPUT_PULLUP);
    pinMode(encb, INPUT_PULLUP);
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
}

void Motor::setMotorSpeed(float phi) {
    // Set the motor speed based on the input value
    int pwr = min(255, max(0, 1.0 * fabs(phi)));
    analogWrite(pwm, pwr);
    digitalWrite(dir, phi >= 0 ? HIGH : LOW);
}

void Motor::readEncoder() {
    pos += digitalRead(encb) ? 1 : -1; // Increment or decrement based on encb state
}
