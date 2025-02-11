#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

class Odometry {
private:
    double x, y, theta;
    double wheel_radius;
    double wheel_base;

public:
    Odometry(double wheel_radius, double wheel_base)
        : x(0), y(0), theta(0),
          wheel_radius(wheel_radius), wheel_base(wheel_base) {}

    void update(double left_ticks, double right_ticks) {
        double left_dist = left_ticks * (2 * PI * wheel_radius) / 360.0;
        double right_dist = right_ticks * (2 * PI * wheel_radius) / 360.0;

        double delta_d = (left_dist + right_dist) / 2.0;
        double delta_theta = (right_dist - left_dist) / wheel_base;

        x += delta_d * cos(theta);
        y += delta_d * sin(theta);
        theta += delta_theta;

        Serial.print("Odometry - X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Theta: ");
        Serial.println(theta);
    }

    double getX() const { return x; }

    double getY() const { return y; }

    double getTheta() const { return theta; }

    void reset() {
        x = 0;
        y = 0;
        theta = 0;
    }
};

#endif // ODOMETRY_H
