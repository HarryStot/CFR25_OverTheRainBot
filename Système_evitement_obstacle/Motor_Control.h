#include <Arduino.h>

class Motor {
private:
  byte enca;
  byte encb;
  byte in1;
  byte in2;

public:
  volatile int pos;

  Motor() {}  //Ne pas utiliser
  Motor(byte enca, byte encb, byte in1, byte in2);

  void init();
  void setMotorSpeed(int phi);
  void readEncoder();
};