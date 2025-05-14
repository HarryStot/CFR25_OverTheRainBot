#include <Wire.h> // Librairie pour la communication I2C
#include <Adafruit_PWMServoDriver.h> // Librairie pour la carte PWM Adafruit
#include <NewPing.h> // Librairie pour le capteur à ultrasons
#include <LiquidCrystal_I2C.h> // Librairie pour l'écran LCD I2C

// Création d'un objet pour contrôler la carte PWM Adafruit
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

// Création d'un objet pour l'écran LCD I2C
LiquidCrystal_I2C lcd(0x20, 16, 2); // Adresse I2C à adapter si nécessaire 20 40 7C ??

int MODE_servo = 0; // 0 pour le mode servo, 1 pour le mode capteur

/* ###################### Ultrason #################*/
// Ce code est adaptable à 1 ou plusieurs capteurs en changeant NB_CAPTEUR et les Pin 
#define NB_CAPTEUR 3     // Number of sensors.
#define MAX_DISTANCE 45 // Maximum distance (in cm) to ping.

//Pin capteur 1
#define ECHO_PIN_1 8
#define TRIG_PIN_1 9
//Pin capteur 2
#define ECHO_PIN_2 10
#define TRIG_PIN_2 11
//Pin capteur 3
#define ECHO_PIN_3 12
#define TRIG_PIN_3 13

float vitesse_son = sqrt(1 + 25 / 273.15) / 60.368;
float mesure;

NewPing sonar[NB_CAPTEUR] = {   // Sensor object array.
  NewPing(TRIG_PIN_1, ECHO_PIN_1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(TRIG_PIN_2, ECHO_PIN_2, MAX_DISTANCE), 
  NewPing(TRIG_PIN_3, ECHO_PIN_3, MAX_DISTANCE)
};


/* ###################### Servo ####################*/
// Définition des limites PWM et paramètres globaux
#define SERVO_MIN  150        // Valeur PWM minimale (position 0°)
#define SERVO_MAX  600        // Valeur PWM maximale (position 180°)
#define START_CMD_SERVO "SRV" // Séquence indiquant le début d'une commande servo
#define START_CMD_LCD "LCD"   // Séquence indiquant le début d'une commande LCD
#define START_CMD_MODE "MODE" // Séquence pour changer de mode
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
String serialBuffer = "";          // Tampon pour les caractères reçus
boolean commandComplete = false;   // Indique si une commande complète a été reçue

// Fonction pour initialiser un servomoteur
void initServo(int index) {
  servoStates[index] = {90, 90, DEFAULT_SPEED, 0}; // Position initiale 90°
  int pulse = map(90, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(index, 0, pulse);
}

void setup() {
  Serial.begin(115200);            // Communication série à 115200 bauds

  pwm.begin();                     // Initialisation carte PWM
  pwm.setPWMFreq(50);              // Fréquence PWM à 50 Hz (standard servos)
  
  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(0, 0);
  lcd.print("Pret a lire...");

  // Initialisation de tous les servomoteurs
  for (int i = 0; i < 16; i++) {
    initServo(i);
  }

  Serial.println("Prêt à recevoir des commandes !");
  Serial.println("Formats disponibles:");
  Serial.println("- Servos: SRV servo1:angle1:vitesse1;servo2:angle2:vitesse2");
  Serial.println("- LCD: LCD Message à afficher");
  Serial.println("- Mode: MODE 0 (servo) ou MODE 1 (capteur)");
}

void loop() {
  // Gérer les entrées série
  handleSerialInput();
  
  // Mode capteur ultrason
  if(MODE_servo == 1) {
    readUltrasonicSensors();
  }
  
  // Mode servo - mise à jour des positions
  if(MODE_servo == 0) {
    updateServos();
  }
}

// Fonction centralisée pour gérer les entrées série
void handleSerialInput() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Si c'est un retour à la ligne, la commande est complète
    if (inChar == '\n') {
      commandComplete = true;
    } else if (inChar != '\r') { // Ignorer les retours chariot
      serialBuffer += inChar;
    }
    
    // Si une commande complète est reçue, la traiter
    if (commandComplete) {
      processSerialCommand(serialBuffer);
      serialBuffer = "";
      commandComplete = false;
    }
  }
}

// Fonction pour traiter les commandes série selon leur préfixe
void processSerialCommand(String command) {
  if (command.startsWith(START_CMD_SERVO)) {
    // Commande pour les servomoteurs
    processServoCommandString(command.substring(strlen(START_CMD_SERVO)));
  } 
  else if (command.startsWith(START_CMD_LCD)) {
    // Commande pour l'écran LCD
    processLCDCommand(command.substring(strlen(START_CMD_LCD)));
  }
  else if (command.startsWith(START_CMD_MODE)) {
    // Commande pour changer de mode
    processModeCommand(command.substring(strlen(START_CMD_MODE)));
  }
  // Possibilité d'ajouter d'autres types de commandes ici
}

// Traitement des commandes pour les servomoteurs
void processServoCommandString(String command) {
  Serial.print("Commande servo reçue: ");
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

// Traitement des commandes pour l'écran LCD
void processLCDCommand(String message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message.substring(0, min(16, message.length())));

  if (message.length() > 16) {
    lcd.setCursor(0, 1);
    lcd.print(message.substring(16, min(32, message.length())));
  }
  
  Serial.print("Message LCD affiché: ");
  Serial.println(message);
}

// Traitement des commandes pour changer de mode
void processModeCommand(String command) {
  command.trim();
  int mode = command.toInt();
  
  if (mode == 0 || mode == 1) {
    MODE_servo = mode;
    Serial.print("Mode changé: ");
    Serial.println(MODE_servo == 0 ? "Servo" : "Capteur");
    
    // Affichage du mode sur l'écran LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mode: ");
    lcd.print(MODE_servo == 0 ? "Servo" : "Capteur");
  } else {
    Serial.println("Mode invalide. Utilisez 0 (servo) ou 1 (capteur)");
  }
}

// Lecture des capteurs ultrasoniques
void readUltrasonicSensors() {
  for (uint8_t i = 0; i < NB_CAPTEUR; i++) {
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    mesure = sonar[i].ping_median(5)*vitesse_son;

    if (mesure > 0 && mesure < 40) {
      // Envoi du numéro de capteur et de la mesure sur le port série
      Serial.print("us,"); // prefix
      Serial.print(i);
      Serial.print(",");
      Serial.println(mesure);
    }
  }
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