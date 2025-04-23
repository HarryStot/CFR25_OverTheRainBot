#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Création d'un objet pour contrôler la carte PWM Adafruit
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

// Définition des limites PWM et paramètres globaux
#define SERVO_MIN  150        // Valeur PWM minimale (position 0°)
#define SERVO_MAX  600        // Valeur PWM maximale (position 180°)
#define START_CMD "SRV"       // Séquence indiquant le début d'une commande
#define END_CMD "\r\n"        // Séquence indiquant la fin d'une commande
#define CMD_SEPARATOR ';'     // Séparateur entre commandes individuelles
#define DEFAULT_SPEED 10      // Vitesse par défaut (degrés par intervalle)
#define UPDATE_INTERVAL 20    // Intervalle de mise à jour en millisecondes

// Structure pour stocker l'état de chaque servomoteur
struct ServoState {
  int currentAngle, targetAngle, speed;
  unsigned long lastUpdate;
};

ServoState servoStates[16];        // États des 16 servomoteurs possibles
String buffer = "";                // Tampon pour les caractères reçus
boolean commandInProgress = false; // Indique si réception en cours
int startCharIndex = 0;            // Pour détecter la séquence de début

// Fonction pour initialiser un servomoteur
void initServo(int index) {
  servoStates[index] = {90, 90, DEFAULT_SPEED, 0}; // Position initiale 90°
  int pulse = map(90, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(index, 0, pulse);
}

void setup() {
  Serial.begin(9600);              // Communication série à 9600 bauds
  pwm.begin();                     // Initialisation carte PWM
  pwm.setPWMFreq(50);              // Fréquence PWM à 50 Hz (standard servos)
  
  // Initialisation de tous les servomoteurs
  for (int i = 0; i < 16; i++) {
    initServo(i);
  }

  Serial.println("Prêt à recevoir des commandes !");
  Serial.println("Format: SRV servo1:angle1:vitesse1;servo2:angle2:vitesse2\\r\\n");
  Serial.println("La vitesse est optionnelle (par défaut: 10)");
}

void loop() {
  // Lecture et traitement des commandes série
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Détection de la séquence de début "SRV"
    if (!commandInProgress) {
      if (inChar == START_CMD[startCharIndex]) {
        startCharIndex++;
        if (startCharIndex == strlen(START_CMD)) {
          buffer = "";
          commandInProgress = true;
          startCharIndex = 0;
        }
      } else {
        startCharIndex = 0; // Réinitialisation si séquence incorrecte
      }
    }
    // Détection de la séquence de fin "\r\n"
    else if (inChar == '\r') {
      // Attendre le '\n' dans la prochaine itération
    }
    else if (inChar == '\n' && commandInProgress) {
      processCommand(buffer);  // Traitement de la commande complète
      commandInProgress = false;
    }
    // Ajout du caractère au buffer si commande en cours
    else if (commandInProgress) {
      buffer += inChar;
    }
  }
  
  // Mise à jour des positions des servomoteurs
  updateServos();
}

// Mise à jour progressive des positions selon vitesse définie
void updateServos() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 16; i++) {
    // Vérifier s'il est temps de mettre à jour ce servo
    if (currentTime - servoStates[i].lastUpdate >= UPDATE_INTERVAL) {
      // Si le servo n'a pas atteint sa position cible
      if (servoStates[i].currentAngle != servoStates[i].targetAngle) {
        // Calculer le déplacement en fonction de la vitesse
        if (servoStates[i].currentAngle < servoStates[i].targetAngle) {
          servoStates[i].currentAngle += min(servoStates[i].speed, 
                                          servoStates[i].targetAngle - servoStates[i].currentAngle);
        } else {
          servoStates[i].currentAngle -= min(servoStates[i].speed, 
                                          servoStates[i].currentAngle - servoStates[i].targetAngle);
        }
        
        // Appliquer la nouvelle position
        int pulse = map(servoStates[i].currentAngle, 0, 180, SERVO_MIN, SERVO_MAX);
        pwm.setPWM(i, 0, pulse);
      }
      servoStates[i].lastUpdate = currentTime;
    }
  }
}

// Traitement d'une série de commandes séparées par des points-virgules
void processCommand(String command) {
  Serial.print("Commande reçue: ");
  Serial.println(command);
  
  int startIndex = 0;
  int endIndex = command.indexOf(CMD_SEPARATOR);
  
  // Si pas de séparateur, traiter comme une seule commande
  if (endIndex == -1) {
    processServoCommand(command);
    return;
  }
  
  // Traiter toutes les commandes séparées
  while (endIndex != -1) {
    processServoCommand(command.substring(startIndex, endIndex));
    startIndex = endIndex + 1;
    endIndex = command.indexOf(CMD_SEPARATOR, startIndex);
  }
  
  // Traiter la dernière commande
  if (startIndex < command.length()) {
    processServoCommand(command.substring(startIndex));
  }
}

// Traitement d'une commande individuelle de servomoteur
void processServoCommand(String servoCmd) {
  int firstSeparator = servoCmd.indexOf(':');
  
  if (firstSeparator != -1) {
    int servo_id = servoCmd.substring(0, firstSeparator).toInt();
    int secondSeparator = servoCmd.indexOf(':', firstSeparator + 1);
    
    int angle, speed;
    
    // Vérifier si la vitesse est spécifiée
    if (secondSeparator != -1) {
      angle = servoCmd.substring(firstSeparator + 1, secondSeparator).toInt();
      speed = servoCmd.substring(secondSeparator + 1).toInt();
      if (speed <= 0) speed = DEFAULT_SPEED;
    } else {
      angle = servoCmd.substring(firstSeparator + 1).toInt();
      speed = DEFAULT_SPEED;
    }

    // Vérifier que les valeurs sont valides
    if (servo_id >= 0 && servo_id < 16 && angle >= 0 && angle <= 180) {
      servoStates[servo_id].targetAngle = angle;
      servoStates[servo_id].speed = speed;
      
      Serial.print("Servo ");
      Serial.print(servo_id);
      Serial.print(" en mouvement vers ");
      Serial.print(angle);
      Serial.print("° à vitesse ");
      Serial.println(speed);
    } else {
      Serial.print("Erreur : valeurs invalides pour ");
      Serial.println(servoCmd);
    }
  } else {
    Serial.print("Erreur : format invalide pour ");
    Serial.println(servoCmd);
  }
}