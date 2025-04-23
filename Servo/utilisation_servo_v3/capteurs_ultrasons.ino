// Capteur 1
const int trigPin1 = 4;
const int echoPin1 = 18;

// Capteur 2
const int trigPin2 = 5;
const int echoPin2 = 19;

// Capteur 3
const int trigPin3 = 6;
const int echoPin3 = 20;

// Variables pour capteur 1
volatile unsigned long startTime1, endTime1;
volatile unsigned long duration1;

// Variables pour capteur 2
volatile unsigned long startTime2, endTime2;
volatile unsigned long duration2;

// Variables pour capteur 3
volatile unsigned long startTime3, endTime3;
volatile unsigned long duration3;

float prevDist1 = 0;
float prevDist2 = 0;
float prevDist3 = 0;

float distance_detect = 40.0;

void setup() {
  Serial.begin(115200);

  // TRIG en sortie
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);

  // ECHO en entrée
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);

  // Interruptions
  attachInterrupt(digitalPinToInterrupt(echoPin1), echo1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin2), echo2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin3), echo3_ISR, CHANGE);
}

void loop() {
  // Déclenchement capteur 1
  digitalWrite(trigPin1, LOW); delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin1, LOW); delay(60);

  // Déclenchement capteur 2
  digitalWrite(trigPin2, LOW); delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin2, LOW); delay(60);

  // Déclenchement capteur 3
  digitalWrite(trigPin3, LOW); delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin3, LOW); delay(60);

  // Lecture sécurisée des mesures
  noInterrupts();
  float dist1 = duration1 / 58.0;
  float dist2 = duration2 / 58.0;
  float dist3 = duration3 / 58.0;
  interrupts();

  // Vérification hors plage
  //bool obstacle1 =  (dist1 < borne_min || dist1 > borne_max);
  bool obstacle1 = (dist1 < distance_detect);
  bool obstacle2 = (dist2 < distance_detect);
  bool obstacle3 = (dist3 < distance_detect);

  if (obstacle1 || obstacle2 || obstacle3) {
    Serial.print("us,1:");
    Serial.print(dist1, 2);
    Serial.print(",2:");
    Serial.print(dist2, 2);
    Serial.print(",3:");
    Serial.println(dist3, 2);
  }
}

// ISR pour capteur 1
void echo1_ISR() {
  if (digitalRead(echoPin1) == HIGH) {
    startTime1 = micros();
  } else {
    endTime1 = micros();
    duration1 = endTime1 - startTime1;
  }
}

// ISR pour capteur 2
void echo2_ISR() {
  if (digitalRead(echoPin2) == HIGH) {
    startTime2 = micros();
  } else {
    endTime2 = micros();
    duration2 = endTime2 - startTime2;
  }
}

// ISR pour capteur 3
void echo3_ISR() {
  if (digitalRead(echoPin3) == HIGH) {
    startTime3 = micros();
  } else {
    endTime3 = micros();
    duration3 = endTime3 - startTime3;
  }
}
