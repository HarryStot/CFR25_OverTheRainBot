#include "Motor_Control.h"
#include "Odometry.h"

#define ENCAD 19
#define ENCBD 18
#define PWMD 3
#define DIRD 12
#define ENCAG 21
#define ENCBG 20
#define PWMG 11
#define DIRG 13

Motor motorR(ENCAD, ENCBD, PWMD, DIRD);
Motor motorL(ENCAG, ENCBG, PWMG, DIRG);
Odometry robot(0.185 / 2, 0.084 / 2);

float x_goal = 0, y_goal = 0, theta_goal = 0;
float K1 = 120;
float eps = 0.05;

void setup() {
    Serial.begin(9600);
    motorR.init();
    motorL.init();
    attachInterrupt(digitalPinToInterrupt(ENCAD), [] { motorR.readEncoder(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCAG), [] { motorL.readEncoder(); }, RISING);
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        x_goal = command.substring(0, firstComma).toFloat();
        y_goal = command.substring(firstComma + 1, secondComma).toFloat();
        theta_goal = command.substring(secondComma + 1).toFloat();
    }

    robot.updateOdometry(motorR.pos, motorL.pos);
    float thetaref = atan2(y_goal - robot.y, x_goal - robot.x);
    float w = K1 * atan2(sin(thetaref - robot.theta), cos(thetaref - robot.theta));
    float v = (abs(thetaref - robot.theta) < eps) ? 10 : 0;

    int speedR = (v + w * robot.L) / robot.r;
    int speedL = (v - w * robot.L) / robot.r;

    motorR.setMotorSpeed(speedR);
    motorL.setMotorSpeed(speedL);

    Serial.print(robot.x); Serial.print(",");
    Serial.print(robot.y); Serial.print(",");
    Serial.println(robot.theta);
    delay(100);
}