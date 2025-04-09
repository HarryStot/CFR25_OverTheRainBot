#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Création d'un objet pour contrôler la carte PWM Adafruit
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

// Définition des limites PWM pour les servomoteurs
#define SERVO_MIN  150  // Valeur PWM minimale (position 0°)
#define SERVO_MAX  600  // Valeur PWM maximale (position 180°)

// Caractères spéciaux pour le format de commande
#define START_CHAR 'SRV'  // Caractère indiquant le début d'une série de commandes
#define END_CHAR '\r\n'    // Caractère indiquant la fin d'une série de commandes
#define CMD_SEPARATOR ';'  // Séparateur entre les commandes individuelles

// Paramètres pour le contrôle de la vitesse
#define DEFAULT_SPEED 10  // Vitesse par défaut (degrés par intervalle)
#define UPDATE_INTERVAL 20  // Intervalle de mise à jour en millisecondes

// Structure pour stocker l'état de chaque servomoteur
struct ServoState {
  int currentAngle;     // Position actuelle en degrés
  int targetAngle;      // Position cible en degrés
  int speed;            // Vitesse en degrés par intervalle
  unsigned long lastUpdate;  // Dernière mise à jour
};

// Tableau pour stocker l'état de chaque servomoteur (max 16 pour la carte Adafruit)
ServoState servoStates[16];

// Variables pour stocker et traiter les commandes entrantes
String buffer = "";     // Tampon pour stocker les caractères reçus
boolean commandInProgress = false;  // Drapeau indiquant si une commande est en cours de réception

void setup() {
    // Initialisation de la communication série à 9600 bauds
    Serial.begin(9600);
    
    // Initialisation de la carte PWM
    pwm.begin();
    // Configuration de la fréquence PWM à 50 Hz (standard pour servomoteurs)
    pwm.setPWMFreq(50);

    // Initialisation de l'état des servomoteurs
    for (int i = 0; i < 16; i++) {
        servoStates[i].currentAngle = 90;  // Position initiale à 90°
        servoStates[i].targetAngle = 90;   // Position cible également à 90°
        servoStates[i].speed = DEFAULT_SPEED;  // Vitesse par défaut
        servoStates[i].lastUpdate = 0;     // Pas encore de mise à jour
        
        // Positionnement initial des servos à 90°
        int pulse = map(90, 0, 180, SERVO_MIN, SERVO_MAX);
        pwm.setPWM(i, 0, pulse);
    }

    // Messages d'information pour l'utilisateur
    Serial.println("Prêt à recevoir des commandes !");
    Serial.println("Format: <servo1:angle1:vitesse1;servo2:angle2:vitesse2;...>");
    Serial.println("La vitesse est optionnelle, en degrés par intervalle (par défaut: 10)");
}

void loop() {
    // Vérification si des données sont disponibles sur le port série
    if (Serial.available()) {
        // Lecture d'un caractère
        char inChar = (char)Serial.read();
        
        // Détection du caractère de début de commande
        if (inChar == START_CHAR) {
            // Réinitialisation du buffer et activation du drapeau
            buffer = "";
            commandInProgress = true;
        }
        // Détection du caractère de fin de commande
        else if (inChar == END_CHAR && commandInProgress) {
            // Traitement de la commande complète
            processCommand(buffer);
            // Désactivation du drapeau
            commandInProgress = false;
        }
        // Ajout du caractère au buffer si une commande est en cours
        else if (commandInProgress) {
            buffer += inChar;
        }
    }
    
    // Mise à jour des positions des servomoteurs avec contrôle de vitesse
    updateServos();
}

// Fonction pour mettre à jour les positions des servomoteurs progressivement
void updateServos() {
    unsigned long currentTime = millis();
    
    // Parcours de tous les servomoteurs
    for (int i = 0; i < 16; i++) {
        // Vérification si c'est le moment de mettre à jour ce servo
        if (currentTime - servoStates[i].lastUpdate >= UPDATE_INTERVAL) {
            // Vérification si le servo a atteint sa position cible
            if (servoStates[i].currentAngle != servoStates[i].targetAngle) {
                // Calcul de la nouvelle position en tenant compte de la vitesse
                if (servoStates[i].currentAngle < servoStates[i].targetAngle) {
                    // Déplacement vers l'avant
                    servoStates[i].currentAngle += min(servoStates[i].speed, 
                                                      servoStates[i].targetAngle - servoStates[i].currentAngle);
                } else {
                    // Déplacement vers l'arrière
                    servoStates[i].currentAngle -= min(servoStates[i].speed, 
                                                      servoStates[i].currentAngle - servoStates[i].targetAngle);
                }
                
                // Envoi de la nouvelle position au servomoteur
                int pulse = map(servoStates[i].currentAngle, 0, 180, SERVO_MIN, SERVO_MAX);
                pwm.setPWM(i, 0, pulse);
            }
            
            // Mise à jour du temps de la dernière modification
            servoStates[i].lastUpdate = currentTime;
        }
    }
}

// Fonction pour traiter une série de commandes
void processCommand(String command) {
    Serial.print("Commande reçue: ");
    Serial.println(command);
    
    // Variables pour parcourir la chaîne
    int startIndex = 0;
    int endIndex = command.indexOf(CMD_SEPARATOR);
    
    // Si pas de séparateur trouvé, c'est la seule commande
    if (endIndex == -1) {
        processServoCommand(command);
        return;
    }
    
    // Traiter toutes les commandes séparées par CMD_SEPARATOR
    while (endIndex != -1) {
        // Extraire la commande individuelle
        String servoCmd = command.substring(startIndex, endIndex);
        processServoCommand(servoCmd);
        
        // Passer à la commande suivante
        startIndex = endIndex + 1;
        endIndex = command.indexOf(CMD_SEPARATOR, startIndex);
    }
    
    // Traiter la dernière commande après le dernier séparateur
    if (startIndex < command.length()) {
        String servoCmd = command.substring(startIndex);
        processServoCommand(servoCmd);
    }
}

// Fonction pour traiter une commande individuelle de servomoteur
void processServoCommand(String servoCmd) {
    // Recherche du premier séparateur (entre ID et angle)
    int firstSeparator = servoCmd.indexOf(':');
    
    // Vérification que le format est correct
    if (firstSeparator != -1) {
        // Extraction de l'ID du servomoteur
        int servo_id = servoCmd.substring(0, firstSeparator).toInt();
        
        // Recherche du deuxième séparateur (entre angle et vitesse)
        int secondSeparator = servoCmd.indexOf(':', firstSeparator + 1);
        
        int angle, speed;
        
        // Vérification si la vitesse est spécifiée
        if (secondSeparator != -1) {
            angle = servoCmd.substring(firstSeparator + 1, secondSeparator).toInt();
            speed = servoCmd.substring(secondSeparator + 1).toInt();
            
            // Si vitesse invalide, utiliser la vitesse par défaut
            if (speed <= 0) {
                speed = DEFAULT_SPEED;
            }
        } else {
            // Pas de vitesse spécifiée, utiliser la valeur par défaut
            angle = servoCmd.substring(firstSeparator + 1).toInt();
            speed = DEFAULT_SPEED;
        }

        // Vérification que les valeurs sont dans les plages acceptables
        if (servo_id >= 0 && servo_id < 16 && angle >= 0 && angle <= 180) {
            // Mise à jour de la position cible et de la vitesse
            servoStates[servo_id].targetAngle = angle;
            servoStates[servo_id].speed = speed;
            
            // Affichage d'un message de confirmation
            Serial.print("Servo ");
            Serial.print(servo_id);
            Serial.print(" en mouvement vers ");
            Serial.print(angle);
            Serial.print("° à vitesse ");
            Serial.println(speed);
        } else {
            // Message d'erreur si les valeurs sont hors limites
            Serial.print("Erreur : valeurs invalides pour ");
            Serial.println(servoCmd);
        }
    } else {
        // Message d'erreur si le format est incorrect
        Serial.print("Erreur : format invalide pour ");
        Serial.println(servoCmd);
    }
}