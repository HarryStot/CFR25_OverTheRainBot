// A implémenter plus tard en tant qu'état du code de la 2e carte arduino (carte qui traite les servo-moteurs)
#include <NewPing.h>

// Ce code est adaptable à 1 ou plusieurs capteurs en changeant NB_CAPTEUR et les Pin 
#define NB_CAPTEUR 3     // Number of sensors.
#define MAX_DISTANCE 45 // Maximum distance (in cm) to ping.

//Pin capteur 1
#define ECHO_PIN_1 18
#define TRIG_PIN_1 4
//Pin capteur 2
#define ECHO_PIN_2 19
#define TRIG_PIN_2 5
//Pin capteur 3
#define ECHO_PIN_3 20
#define TRIG_PIN_3 6

float vitesse_son = sqrt(1 + 25 / 273.15) / 60.368;
float mesure;

NewPing sonar[NB_CAPTEUR] = {   // Sensor object array.
  NewPing(TRIG_PIN_1, ECHO_PIN_1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(TRIG_PIN_2, ECHO_PIN_2, MAX_DISTANCE), 
  NewPing(TRIG_PIN_3, ECHO_PIN_3, MAX_DISTANCE)
};

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() { 
  for (uint8_t i = 0; i < NB_CAPTEUR; i++) { // Loop through each sensor and display results.
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