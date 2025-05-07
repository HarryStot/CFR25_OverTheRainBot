#include "Motor_Control.h"

Motor::Motor(byte enca, byte encb, byte pwm, byte dir) {
    this->enca = enca;
    this->encb = encb;
    this->pwm = pwm;
    this->dir = dir;

    this->lastUpdateTime = 0;
    this->currentPwr = 0;
}

void Motor::init() {
    pinMode(enca, INPUT_PULLUP);
    pinMode(encb, INPUT_PULLUP);
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
}

void Motor::setMotorSpeed(float phi) {
    unsigned long currentTime = millis();
    int targetPwr = min(255, max(0, int(1.0 * fabs(phi))));
    bool direction = phi >= 0;

    if (currentTime - lastUpdateTime >= 10) { // mise à jour toutes les 10 ms
        lastUpdateTime = currentTime;

        // changer la direction du moteur
        digitalWrite(dir, direction ? HIGH : LOW);

        if (targetPwr == 0) {
            // Si la consigne est zéro -> arrêt immédiat
            currentPwr = 0;
        } else {
            // Sinon, montée progressive
            if (currentPwr < targetPwr) {
                currentPwr += 10;
                if (currentPwr > targetPwr) currentPwr = targetPwr;
            }
            // (Pas de descente progressive)
        }

        analogWrite(pwm, currentPwr);
    }
}

void Motor::readEncoder() {
    pos += digitalRead(encb) ? 1 : -1;
}
