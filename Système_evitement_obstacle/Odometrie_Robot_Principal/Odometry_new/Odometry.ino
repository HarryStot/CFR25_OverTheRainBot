#include "Motor_Control.h"
#include "Odometry.h"

#define ENCAD 19
#define ENCBD 18
#define PWMD 11
#define DIRD 13
#define ENCAG 21
#define ENCBG 20
#define PWMG 3
#define DIRG 12

Motor motorR(ENCAD, ENCBD, PWMD, DIRD);
Motor motorL(ENCAG, ENCBG, PWMG, DIRG);
Odometry robot(0.245 / 2, 0.065 / 2);

void setup() {
    Serial.begin(9600);
    motorR.init();
    attachInterrupt(digitalPinToInterrupt(ENCAD), [] { motorR.readEncoder(); }, RISING);
    motorL.init();
    attachInterrupt(digitalPinToInterrupt(ENCAG), [] { motorL.readEncoder(); }, RISING);
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        int speedR = command.substring(0, command.indexOf(',')).toInt();
        int speedL = command.substring(command.indexOf(',') + 1).toInt();
        motorR.setMotorSpeed(speedR);
        motorL.setMotorSpeed(speedL);
    }

    robot.updateOdometry(motorR.pos, motorL.pos);
    Serial.print(robot.x); Serial.print(",");
    Serial.print(robot.y); Serial.print(",");
    Serial.println(robot.theta);
    delay(100);
}
