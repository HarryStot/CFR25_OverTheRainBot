#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

#define SERVO_MIN  150  // Valeur PWM min pour ton servo
#define SERVO_MAX  600  // Valeur PWM max pour ton servo

void setup() {
    Serial.begin(9600);  // Port série pour la communication
    pwm.begin();
    pwm.setPWMFreq(50);  // Fréquence pour les servos (50 Hz)

    Serial.println("Prêt à recevoir des commandes !");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Lire jusqu'à nouvelle ligne
        input.trim();  // Supprime les espaces et sauts de ligne

        // Vérifie que la commande est bien du type "servo_id:angle"
        int separatorIndex = input.indexOf(':');
        if (separatorIndex != -1) {
            int servo_id = input.substring(0, separatorIndex).toInt();
            int angle = input.substring(separatorIndex + 1).toInt();

            // Vérifie que les valeurs sont valides
            if (servo_id >= 0 && servo_id < 16 && angle >= 0 && angle <= 180) {
                int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
                pwm.setPWM(servo_id, 0, pulse);
                Serial.print("Servo ");
                Serial.print(servo_id);
                Serial.print(" réglé à ");
                Serial.println(angle);
            } else {
                Serial.println("Erreur : valeurs invalides !");
            }
        } else {
            Serial.println("Erreur : format invalide !");
        }
    }
}
