// StepperMotorControl.cpp
#include "Driver_TB_mini.h"

// Variable statique pour la gestion des erreurs
static StepperMotorControl* currentInstance = nullptr;

// Constructeur
StepperMotorControl::StepperMotorControl(
  uint8_t vrefPin, uint8_t mode0Pin, uint8_t mode1Pin, 
  uint8_t mode2Pin, uint8_t dirPin, uint8_t clkPin,
  uint8_t standbyPin, uint8_t enablePin, uint8_t resetPin,
  uint8_t lo0Pin, uint8_t lo1Pin, uint8_t agcPin,
  uint8_t clim0Pin, uint8_t clim1Pin, uint8_t flimPin,
  float stepAngle
) : 
  _vrefPin(vrefPin),
  _mode0Pin(mode0Pin),
  _mode1Pin(mode1Pin),
  _mode2Pin(mode2Pin),
  _dirPin(dirPin),
  _clkPin(clkPin),
  _standbyPin(standbyPin),
  _enablePin(enablePin),
  _resetPin(resetPin),
  _lo0Pin(lo0Pin),
  _lo1Pin(lo1Pin),
  _agcPin(agcPin),
  _clim0Pin(clim0Pin),
  _clim1Pin(clim1Pin),
  _flimPin(flimPin),
  _stepAngle(stepAngle)
{
  // Créer une instance d'AccelStepper
  _stepper = new AccelStepper(AccelStepper::DRIVER, _clkPin, _dirPin);
  
  // Stocker l'instance pour les interruptions
  currentInstance = this;
}

// Destructeur
StepperMotorControl::~StepperMotorControl() {
  // Libérer la mémoire allouée à AccelStepper
  delete _stepper;
}

// Méthode d'initialisation
void StepperMotorControl::init() {
  // Configuration des broches en sortie
  pinMode(_mode0Pin, OUTPUT);
  pinMode(_mode1Pin, OUTPUT);
  pinMode(_mode2Pin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_clkPin, OUTPUT);
  pinMode(_standbyPin, OUTPUT);
  pinMode(_enablePin, OUTPUT);
  pinMode(_resetPin, OUTPUT);
  pinMode(_lo0Pin, INPUT);
  pinMode(_lo1Pin, INPUT);
  pinMode(_agcPin, OUTPUT);
  pinMode(_clim0Pin, OUTPUT);
  pinMode(_clim1Pin, OUTPUT);
  pinMode(_flimPin, OUTPUT);

  // Configuration par défaut des microstepping (MODE0=LOW, MODE1=HIGH, MODE2=LOW)
  configureMicrostepping(LOW, HIGH, LOW);
  
  // Rotation par défaut (CCW)
  digitalWrite(_dirPin, LOW);
  
  // Activation du driver et moteur
  digitalWrite(_standbyPin, HIGH);  // Activer le driver
  digitalWrite(_enablePin, HIGH);   // Activer les sorties moteurs
  
  // Activer ACG (Active Control Gain)
  digitalWrite(_agcPin, HIGH);
  
  // Configuration des limites de courant par défaut
  digitalWrite(_clim0Pin, LOW);
  
  // Réinitialisation de l'angle électrique
  digitalWrite(_resetPin, HIGH);
  delay(10);
  digitalWrite(_resetPin, LOW);
  
  // Configuration des interruptions pour la détection d'erreurs
  attachInterrupt(digitalPinToInterrupt(_lo0Pin), StepperMotorControl::handleError, FALLING);
  attachInterrupt(digitalPinToInterrupt(_lo1Pin), StepperMotorControl::handleError, FALLING);
  
  // Configuration initiale d'AccelStepper
  _stepper->setMaxSpeed(1000);
  _stepper->setAcceleration(500);
  _stepper->setCurrentPosition(0);
}

// Configuration des microstepping
void StepperMotorControl::configureMicrostepping(bool mode0, bool mode1, bool mode2) {
  digitalWrite(_mode0Pin, mode0);
  digitalWrite(_mode1Pin, mode1);
  digitalWrite(_mode2Pin, mode2);
}

// Déplacement du moteur
void StepperMotorControl::moveToPosition(int speedRpm, float targetDegrees) {
  // Activer le moteur et le driver
  turnOn();
  
  // Calculer la vitesse en pas/seconde
  float stepsPerRevolution = round(360.0 / _stepAngle);
  float speedInStepsPerSec = (speedRpm / 60.0) * stepsPerRevolution * 4;
  
  // Configurer la vitesse et l'accélération
  _stepper->setMaxSpeed(speedInStepsPerSec);
  _stepper->setAcceleration(speedInStepsPerSec * 1.5);  // Accélération pour atteindre Vmax en 0.5 secondes
  
  // Calculer le nombre de pas à effectuer
  float targetSteps = round(targetDegrees / _stepAngle) * 4;
  
  // Définir la position cible
  _stepper->moveTo(targetSteps);
  _stepper->run();
}

// Exécuter un pas (non bloquant)
void StepperMotorControl::run() {
  _stepper->run();
}

// Exécuter jusqu'à la position cible (bloquant)
void StepperMotorControl::runToPosition() {
  _stepper->runToPosition();
}

// Activer le moteur
void StepperMotorControl::turnOn() {
  digitalWrite(_enablePin, HIGH);
  digitalWrite(_standbyPin, HIGH);
}

// Désactiver le moteur
void StepperMotorControl::turnOff() {
  digitalWrite(_enablePin, LOW);
}

// Réinialiser la position
void StepperMotorControl::resetPosition() {
  _stepper->setCurrentPosition(0);
}

// Gestionnaire d'erreurs statique
void StepperMotorControl::handleError() {
  if (currentInstance) {
    // Désactiver le moteur et mettre le driver en veille
    digitalWrite(currentInstance->_enablePin, LOW);
    digitalWrite(currentInstance->_standbyPin, LOW);
    Serial.println("Erreur détectée ; Arrêt moteur et driver");
  }
}