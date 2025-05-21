// StepperMotorControl.h
#ifndef STEPPER_MOTOR_CONTROL_H
#define STEPPER_MOTOR_CONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>

class StepperMotorControl {
  private:
    // Broches du driver TB67S128FTG
    uint8_t _vrefPin;
    uint8_t _mode0Pin;
    uint8_t _mode1Pin;
    uint8_t _mode2Pin;
    uint8_t _dirPin;
    uint8_t _clkPin;
    uint8_t _standbyPin;
    uint8_t _enablePin;
    uint8_t _resetPin;
    uint8_t _lo0Pin;
    uint8_t _lo1Pin;
    uint8_t _agcPin;
    uint8_t _clim0Pin;
    uint8_t _clim1Pin;
    uint8_t _flimPin;
    
    // Paramètres du moteur
    float _stepAngle;
    
    // Objet AccelStepper
    AccelStepper* _stepper;
    
    // Méthode privée pour la gestion des erreurs
    static void handleError();
    
  public:
    // Constructeur avec paramètres par défaut
    StepperMotorControl(
      uint8_t vrefPin = A0,
      uint8_t mode0Pin = 26,
      uint8_t mode1Pin = 28,
      uint8_t mode2Pin = 30,
      uint8_t dirPin = 32,
      uint8_t clkPin = 34,
      uint8_t standbyPin = 36,
      uint8_t enablePin = 38,
      uint8_t resetPin = 40,
      uint8_t lo0Pin = 44,
      uint8_t lo1Pin = 46,
      uint8_t agcPin = 50,
      uint8_t clim0Pin = 51,
      uint8_t clim1Pin = 52,
      uint8_t flimPin = 53,
      float stepAngle = 0.9
    );
    
    // Destructeur
    ~StepperMotorControl();
    
    // Méthodes d'initialisation
    void init();
    void configureMicrostepping(bool mode0, bool mode1, bool mode2);
    
    // Méthodes de contrôle du moteur
    void moveToPosition(int speedRpm, float targetDegrees);
    void run();
    void runToPosition();
    void turnOn();
    void turnOff();
    void resetPosition();
    
    // Accesseurs
    AccelStepper* getStepper() { return _stepper; }
};

#endif // STEPPER_MOTOR_CONTROL_H