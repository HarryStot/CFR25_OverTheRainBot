#include <Arduino.h>

class Motor {
private:
  float u;
  byte pwm;
  byte enca;
  byte encb;
  byte dir;

public:
  volatile int pos;

  Motor() {}  //Ne pas utiliser
  Motor(byte enca, byte encb, byte pwm, byte dir);

  void init();
  void setMotorSpeed(int phi);
  void readEncoder();
};