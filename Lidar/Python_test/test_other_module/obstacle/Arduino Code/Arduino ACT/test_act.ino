/**
 * Arduino controller for servo and stepper motors with sensor reading
 */

#include <Servo.h>
#include <AccelStepper.h>

// Pins
const int SERVO_PIN = 9;        // Pour le servomoteur
const int SENSOR_PIN = A0;      // Entrée analogique pour le capteur

// Configuration du stepper
#define STEPPER_PIN1 4
#define STEPPER_PIN2 5
#define STEPPER_PIN3 6
#define STEPPER_PIN4 7

// Matériel
Servo motorServo;
AccelStepper stepper(AccelStepper::FULL4WIRE, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

// Traitement des commandes série
String inputBuffer = "";
bool commandComplete = false;

void setup() {
  // Initialisation de la communication série
  Serial.begin(115200);

  // Initialisation du servomoteur
  motorServo.attach(SERVO_PIN);
  motorServo.write(90); // Position neutre
  Serial.println("Servo motor initialized");

  // Initialisation du stepper
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  Serial.println("Stepper motor initialized");

  Serial.println("System ready to receive commands");
}

void loop() {
  // Vérification des nouvelles commandes
  receiveSerialCommands();

  // Mise à jour du stepper
  stepper.run();
}

void receiveSerialCommands() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (inputBuffer.length() > 0) {
        commandComplete = true;
      }
    } else {
      inputBuffer += inChar;
    }
  }

  if (commandComplete) {
    processCommand(inputBuffer);
    inputBuffer = "";
    commandComplete = false;
  }
}

void processCommand(String command) {
  command.trim();

  if (command.startsWith("SERVO")) {
    // Format: "SERVO,X" où X est l'angle (0-180)
    int commaIndex = command.indexOf(",");
    if (commaIndex > 0) {
      int value = command.substring(commaIndex + 1).toInt();
      value = constrain(value, 0, 180);
      motorServo.write(value);
      Serial.print("Servo moved to position: ");
      Serial.println(value);
    }
  }
  else if (command.startsWith("STEPPER")) {
    // Format: "STEPPER,X" où X est la position cible
    int commaIndex = command.indexOf(",");
    if (commaIndex > 0) {
      int value = command.substring(commaIndex + 1).toInt();
      stepper.moveTo(value);
      Serial.print("Stepper moving to position: ");
      Serial.println(value);
    }
  }
  else if (command.startsWith("MOVE")) {
    // Pour compatibilité avec ancien code
    // MOVE,X déplacera le servomoteur par défaut
    int commaIndex = command.indexOf(",");
    if (commaIndex > 0) {
      int value = command.substring(commaIndex + 1).toInt();
      value = constrain(value, 0, 180);
      motorServo.write(value);
      Serial.print("Servo moved to position: ");
      Serial.println(value);
    }
  }
  else if (command.startsWith("READ")) {
    // Lire la valeur du capteur
    int sensorValue = analogRead(SENSOR_PIN);
    Serial.print("Sensor value: ");
    Serial.println(sensorValue);
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}