#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <NewPing.h>
#include "Driver_TB_mini.h"
//#include <LiquidCrystal_I2C.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//LiquidCrystal_I2C lcd(0x20, 16, 2);

int MODE_servo = 0;

/* ###################### Ultrason ################## */
#define NB_CAPTEUR 3
#define MAX_DISTANCE 45

#define ECHO_PIN_1 12
#define TRIG_PIN_1 13
#define ECHO_PIN_2 11
#define TRIG_PIN_2 10
#define ECHO_PIN_3 9
#define TRIG_PIN_3 8

// Création d'une instance de StepperMotorControl avec les valeurs par défaut
StepperMotorControl motorControl;




float vitesse_son = sqrt(1 + 25 / 273.15) / 60.368;
float mesure;

NewPing sonar[NB_CAPTEUR] = {
  NewPing(TRIG_PIN_1, ECHO_PIN_1, MAX_DISTANCE),
  NewPing(TRIG_PIN_2, ECHO_PIN_2, MAX_DISTANCE),
  NewPing(TRIG_PIN_3, ECHO_PIN_3, MAX_DISTANCE)
};

/* ###################### Servo #################### */
#define SERVO_MIN 150
#define SERVO_MAX 600
#define START_CMD_SERVO "SRV"
#define START_CMD_LCD "LCD"
#define START_CMD_MODE "MODE"
#define CMD_SEPARATOR ';'
#define DEFAULT_SPEED 10
#define UPDATE_INTERVAL 20

struct ServoState {
  int currentAngle, targetAngle, speed;
  unsigned long lastUpdate;
};

ServoState servoStates[16];
String serialBuffer = "";
boolean commandComplete = false;

void initServo(int index) {
  servoStates[index] = { 90, 90, DEFAULT_SPEED, 0 };
  int pulse = map(90, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(index, 0, pulse);
}


void setup() {
  delay(2000);
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  motorControl.init();
  Serial.println("Initialisation du stepper terminée");

  //lcd.init();
  //lcd.backlight();
  //lcd.setCursor(0, 0);
  //lcd.print("Pret a lire...");

  //for (int i = 0; i < 15; i++) {  // J'ai mis 15 et pas 16 Parce que l'initialisation du sevro de la grande pince ne marche pas
  //  initServo(i);
  //}

  Serial.println("Prêt à recevoir des commandes !");
  Serial.println("- Servos: SRV servo:angle:vitesse");
  Serial.println("- LCD: LCDL1:texte1;L2:texte2");
  Serial.println("- Mode: MODE 0 ou MODE 1");
}


void loop() {
  handleSerialInput();

  if (MODE_servo == 1) {
    readUltrasonicSensors();
  }

  if (MODE_servo == 0) {
    updateServos();
  }

  motorControl.run();  //Moteur non bloquant
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      commandComplete = true;
    } else if (inChar != '\r') {
      serialBuffer += inChar;
    }

    if (commandComplete) {
      processSerialCommand(serialBuffer);
      serialBuffer = "";
      commandComplete = false;
    }
  }
}

void processSerialCommand(String command) {
  if (command.startsWith(START_CMD_SERVO)) {
    processServoCommandString(command.substring(strlen(START_CMD_SERVO)));
  } else if (command.startsWith(START_CMD_LCD)) {
    processLCDCommand(command.substring(strlen(START_CMD_LCD)));
  } else if (command.startsWith(START_CMD_MODE)) {
    processModeCommand(command.substring(strlen(START_CMD_MODE)));
  } else if (command.startsWith("STEP:") || command.startsWith("STEP :")) {
    // Supprimer tous les espaces pour éviter les erreurs de parsing
    command.replace(" ", "");
    command.replace("STEP:", "");  // Supprimer le préfixe
    command.trim();

    int separatorIndex = command.indexOf(':');
    if (separatorIndex != -1) {
      String speedStr = command.substring(0, separatorIndex);
      String angleStr = command.substring(separatorIndex + 1);

      speedStr.trim();
      angleStr.trim();

      int speed = speedStr.toInt();
      int angle = angleStr.toInt();

      // Vérification des plages de valeurs
      if (speed > 0 && speed <= 100 && angle >= 0 && angle <= 1080) {
        Serial.print("Commande stepper reçue -> Vitesse : ");
        Serial.print(speed);
        Serial.print(" RPM, Angle : ");
        Serial.print(angle);
        Serial.println("°");

        motorControl.moveToPosition(speed, angle);
      } else {
        Serial.println("Erreur : Vitesse ou angle invalide.");
      }
    } else {
      Serial.println("Format invalide. Utilisez : STEP:x:y");
    }
  }
}

void processServoCommandString(String command) {
  Serial.print("Commande servo reçue: ");
  Serial.println(command);

  int startIndex = 0;
  int endIndex = command.indexOf(CMD_SEPARATOR);

  if (endIndex == -1) {
    processServoCommand(command);
    return;
  }

  while (endIndex != -1) {
    processServoCommand(command.substring(startIndex, endIndex));
    startIndex = endIndex + 1;
    endIndex = command.indexOf(CMD_SEPARATOR, startIndex);
  }

  if (startIndex < command.length()) {
    processServoCommand(command.substring(startIndex));
  }
}

void processServoCommand(String servoCmd) {
  int firstSeparator = servoCmd.indexOf(':');

  if (firstSeparator != -1) {
    int servo_id = servoCmd.substring(0, firstSeparator).toInt();
    int secondSeparator = servoCmd.indexOf(':', firstSeparator + 1);

    int angle, speed;
    if (secondSeparator != -1) {
      angle = servoCmd.substring(firstSeparator + 1, secondSeparator).toInt();
      speed = servoCmd.substring(secondSeparator + 1).toInt();
      if (speed <= 0) speed = DEFAULT_SPEED;
    } else {
      angle = servoCmd.substring(firstSeparator + 1).toInt();
      speed = DEFAULT_SPEED;
    }

    if (servo_id >= 0 && servo_id < 16 && angle >= 0 && angle <= 180) {
      servoStates[servo_id].targetAngle = angle;
      servoStates[servo_id].speed = speed;
      Serial.print("Servo ");
      Serial.print(servo_id);
      Serial.print(" -> ");
      Serial.print(angle);
      Serial.print("° à ");
      Serial.print(speed);
      Serial.println(" vitesse");
    } else {
      Serial.print("Erreur : valeurs invalides : ");
      Serial.println(servoCmd);
    }
  } else {
    Serial.print("Erreur de format : ");
    Serial.println(servoCmd);
  }
}

void processLCDCommand(String message) {
  String line1 = "";
  String line2 = "";

  int l1Index = message.indexOf("L1:");
  int l2Index = message.indexOf("L2:");

  if (l1Index != -1) {
    int endL1 = (l2Index != -1) ? l2Index - 1 : message.length();
    line1 = message.substring(l1Index + 3, endL1);
  }

  if (l2Index != -1) {
    line2 = message.substring(l2Index + 3);
  }

  //lcd.clear();
  //lcd.setCursor(0, 0);
  //lcd.print(line1.substring(0, min(16, line1.length())));
  //lcd.setCursor(0, 1);
  //lcd.print(line2.substring(0, min(16, line2.length())));

  Serial.print("LCD L1: ");
  Serial.println(line1);
  Serial.print("LCD L2: ");
  Serial.println(line2);
}

void processModeCommand(String command) {
  command.trim();
  int mode = command.toInt();

  if (mode == 0 || mode == 1) {
    MODE_servo = mode;
    Serial.print("Mode changé: ");
    Serial.println(MODE_servo == 0 ? "Servo" : "Capteur");

    //lcd.clear();
    //lcd.setCursor(0, 0);
    //lcd.print("Mode: ");
    //lcd.print(MODE_servo == 0 ? "Servo" : "Capteur");
  } else {
    Serial.println("Mode invalide. Utilisez 0 (servo) ou 1 (capteur)");
  }
}

void readUltrasonicSensors() {
  for (uint8_t i = 0; i < NB_CAPTEUR; i++) {
    delay(50);
    mesure = sonar[i].ping_median(5) * vitesse_son;

    if (mesure > 0 && mesure < 40) {
      Serial.print("us,");
      Serial.print(i);
      Serial.print(",");
      Serial.println(mesure);
    }
  }
}

void updateServos() {
  unsigned long currentTime = millis();

  for (int i = 0; i < 16; i++) {
    if (currentTime - servoStates[i].lastUpdate >= UPDATE_INTERVAL) {
      if (servoStates[i].currentAngle != servoStates[i].targetAngle) {
        if (servoStates[i].currentAngle < servoStates[i].targetAngle) {
          servoStates[i].currentAngle += min(servoStates[i].speed, servoStates[i].targetAngle - servoStates[i].currentAngle);
        } else {
          servoStates[i].currentAngle -= min(servoStates[i].speed, servoStates[i].currentAngle - servoStates[i].targetAngle);
        }

        int pulse = map(servoStates[i].currentAngle, 0, 180, SERVO_MIN, SERVO_MAX);
        pwm.setPWM(i, 0, pulse);
      }
      servoStates[i].lastUpdate = currentTime;
    }
  }
}

//void loop() {
// Déplacer à 500 tr/min jusqu'à 100 degrés
// motorControl.moveToPosition(100,1000);
// motorControl.runToPosition();  // Bloquant jusqu'à ce que la position soit atteinte
///delay(10000);
// Retourner à la position initiale (0 degrés)
// motorControl.moveToPosition(100,-300);
//motorControl.runToPosition();  // Bloquant jusqu'à ce que la position soit atteinte

// Éteindre le moteur
//// motorControl.turnOff(); //S'il est en commentaire, il bloque la position
// while (1){}
//}
