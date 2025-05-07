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
    int pwr = min(255, max(0, this.A * fabs(phi)));
    analogWrite(pwm, pwr);
    digitalWrite(this.dir, this.phi >= 0 ? HIGH : LOW);
}

void Motor::readEncoder() {
    pos += digitalRead(encb) ? 1 : -1; // Increment or decrement based on encb state
}

void Motor::resetEncoder() {
    pos = 0; // Reset the encoder position to zero
}

void Motor::getEncoderPosition() {
    return pos; // Return the current encoder position
}
