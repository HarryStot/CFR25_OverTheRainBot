#include <Arduino.h>

class Odometry {
private:

public:
  float pi = 3.14, L, r;
  float posL = 0, posR = 0, theta_prec = pi / 2, theta = 0, x = 0, y = 0, DR = 0, DL = 0, DC = 0, x_prec = 0, y_prec = 0, posL_prec = 0, posR_prec = 0;

  Odometry() {}  //Ne pas utiliser
  Odometry(float L, float r);

  void updateOdometry(float posR, float posL, float x_goal, float y_goal);
};