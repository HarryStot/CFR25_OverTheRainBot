
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Création d'un objet pour contrôler la carte PWM Adafruit
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

// Définition des limites PWM pour les servomoteurs
#define SERVO_MIN  150  // Valeur PWM minimale (position 0°)
#define SERVO_MAX  600  // Valeur PWM maximale (position 180°)

// Caractères spéciaux pour le format de commande
#define START_CHAR '<'  // Caractère indiquant le début d'une série de commandes
#define END_CHAR '>'    // Caractère indiquant la fin d'une série de commandes
#define CMD_SEPARATOR ';'  // Séparateur entre les commandes individuelles

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

    // Messages d'information pour l'utilisateur
    Serial.println("Prêt à recevoir des commandes !");
    Serial.println("Format: <servo1:angle1;servo2:angle2;...>");
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
    // Recherche du séparateur entre ID et angle
    int separatorIndex = servoCmd.indexOf(':');
    
    // Vérification que le format est correct
    if (separatorIndex != -1) {
        // Extraction de l'ID du servomoteur et de l'angle
        int servo_id = servoCmd.substring(0, separatorIndex).toInt();
        int angle = servoCmd.substring(separatorIndex + 1).toInt();

        // Vérification que les valeurs sont dans les plages acceptables
        if (servo_id >= 0 && servo_id < 16 && angle >= 0 && angle <= 180) {
            // Conversion de l'angle (0-180) en valeur PWM (SERVO_MIN-SERVO_MAX)
            int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
            // Envoi de la commande au servomoteur
            pwm.setPWM(servo_id, 0, pulse);
            // Affichage d'un message de confirmation
            Serial.print("Servo ");
            Serial.print(servo_id);
            Serial.print(" réglé à ");
            Serial.println(angle);
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